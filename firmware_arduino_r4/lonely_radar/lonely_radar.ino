/* ============================================================
 * lonely_radar — LiDAR sweep + servo + mood buzzer
 * Target: Arduino Uno R4 (Minima or WiFi)
 *
 * Hardware wiring
 * ---------------
 * LIDAR-Lite v3:
 *   Pin 1 (Power)        -> 5V
 *   Pin 2 (GND)          -> GND
 *   Pin 3 (Mode control) -> unconnected
 *   Pin 4 (SCL)          -> SCL
 *   Pin 5 (SDA)          -> SDA
 *   Pin 6 (Power enable) -> 5V
 *
 * Servo:
 *   Signal wire          -> D9
 *   Power                -> 5V
 *   GND                  -> GND
 *
 * Passive buzzer:
 *   (+) leg              -> D8
 *   (−) leg              -> GND
 * ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "LidarLiteV3.h"

/* ---- Pin assignments ---- */
#define SERVO_PIN        9
#define SPEAKER_PIN      8

/* ---- Sweep parameters ---- */
#define SWEEP_MIN_DEG    45    /* 90° window centred on straight-ahead */
#define SWEEP_MAX_DEG    135
#define SWEEP_STEP_DEG   2
#define STEP_DELAY_MS    12    /* time between servo steps (ms) */

/* ---- Distance thresholds (cm) — 0 is ignored (sensor error) ---- */
#define DIST_CLOSE_CM    60
#define DIST_NEAR_CM     120
#define DIST_FAR_CM      200
#define DIST_MAX_CM      300   /* treat anything >= this as "nobody" */

/* ---- Tone gap between notes ---- */
#define GAP_MS           40

/* ============================================================
 * Mood identifiers
 * ============================================================ */
#define MOOD_LONELY   0
#define MOOD_SAD      1
#define MOOD_HOPEFUL  2
#define MOOD_HAPPY    3
#define MOOD_EXCITED  4
#define MOOD_COUNT    5

/* ============================================================
 * Note tables
 * Each entry: { freq_hz, dur_ms }  — freq == 0 means REST
 *
 * On ARM (Renesas RA4M1) const data already lives in flash;
 * no PROGMEM / pgm_read_* machinery is needed or used.
 * ============================================================ */
struct Note {
    uint16_t freq;
    uint16_t dur_ms;
};

/* LONELY — slow, low, minor, descending, mournful */
static const Note lonelyNotes[] = {
    { 220, 600 },   /* A3 */
    { 196, 600 },   /* G3 */
    { 165, 700 },   /* E3 */
    {   0, 400 },   /* REST */
};

/* SAD — slow minor, slight movement */
static const Note sadNotes[] = {
    { 220, 500 },   /* A3 */
    { 247, 400 },   /* B3 */
    { 196, 500 },   /* G3 */
    {   0, 300 },   /* REST */
};

/* HOPEFUL — moderate, minor→major hint */
static const Note hopefulNotes[] = {
    { 294, 350 },   /* D4 */
    { 330, 300 },   /* E4 */
    { 392, 350 },   /* G4 */
    { 440, 300 },   /* A4 */
};

/* HAPPY — upbeat major */
static const Note happyNotes[] = {
    {  523, 200 },  /* C5 */
    {  659, 180 },  /* E5 */
    {  784, 200 },  /* G5 */
    { 1047, 250 },  /* C6 */
};

/* EXCITED — fast, high, major arpeggio */
static const Note excitedNotes[] = {
    {  523, 100 },  /* C5 */
    {  659,  90 },  /* E5 */
    {  784,  90 },  /* G5 */
    { 1047, 100 },  /* C6 */
    {  784,  90 },  /* G5 */
    {  659,  90 },  /* E5 */
};

/* Dispatch table — indexed by MOOD_* */
static const Note * const moodTables[MOOD_COUNT] = {
    lonelyNotes,
    sadNotes,
    hopefulNotes,
    happyNotes,
    excitedNotes,
};

static const uint8_t moodTableSizes[MOOD_COUNT] = {
    4,  /* LONELY  */
    4,  /* SAD     */
    4,  /* HOPEFUL */
    4,  /* HAPPY   */
    6,  /* EXCITED */
};

/* ============================================================
 * Objects
 * ============================================================ */
static LidarLiteV3 lidar;
static Servo        servo;

/* ============================================================
 * State variables — all static/global, no heap allocation
 * ============================================================ */

/* Servo sweep */
static int      servoAngle  = SWEEP_MIN_DEG;
static int      servoDir    = +SWEEP_STEP_DEG;  /* flips at limits */
static uint32_t lastStepMs  = 0;

/* LiDAR — rolling minimum over the last full sweep */
static uint16_t closestCm   = DIST_MAX_CM;  /* best valid reading this sweep */
static uint16_t displayCm   = DIST_MAX_CM;  /* committed after each full sweep */

/* Tone sequencer */
static uint8_t  noteIndex   = 0;
static uint32_t noteEndMs   = 0;
static int      activeMood  = MOOD_LONELY;  /* mood currently playing */

/* ============================================================
 * Helper: map displayCm to a mood
 * ============================================================ */
static int distanceToMood(uint16_t cm)
{
    if (cm >= DIST_MAX_CM)   return MOOD_LONELY;
    if (cm >= DIST_FAR_CM)   return MOOD_SAD;
    if (cm >= DIST_NEAR_CM)  return MOOD_HOPEFUL;
    if (cm >= DIST_CLOSE_CM) return MOOD_HAPPY;
    return MOOD_EXCITED;
}

/* ============================================================
 * Helper: human-readable mood label
 * ============================================================ */
static const char *moodLabel(int mood)
{
    switch (mood) {
        case MOOD_LONELY:  return "LONELY";
        case MOOD_SAD:     return "SAD";
        case MOOD_HOPEFUL: return "HOPEFUL";
        case MOOD_HAPPY:   return "HAPPY";
        case MOOD_EXCITED: return "EXCITED";
        default:           return "UNKNOWN";
    }
}

/* ============================================================
 * setup
 * ============================================================ */
void setup()
{
    Serial.begin(115200);
    while (!Serial) {}   /* wait for USB CDC on R4 */

    Serial.println(F("lonely_radar — starting up"));

    servo.attach(SERVO_PIN);
    servo.write(SWEEP_MIN_DEG);

    if (!lidar.begin()) {
        Serial.println(F("ERROR: LIDAR not found (I2C 0x62). Check wiring."));
        while (1) {}
    }
    Serial.println(F("LIDAR ready — beginning sweep"));

    lastStepMs = millis();
    noteEndMs  = millis();  /* fire first note immediately */
}

/* ============================================================
 * loop — non-blocking state machine, no delay() calls
 * ============================================================ */
void loop()
{
    uint32_t now = millis();

    /* ----------------------------------------------------------
     * Servo + LiDAR update — runs every STEP_DELAY_MS
     * ---------------------------------------------------------- */
    if (now - lastStepMs >= (uint32_t)STEP_DELAY_MS) {
        lastStepMs = now;

        /* Move servo to current angle */
        servo.write(servoAngle);

        /* Fire LiDAR reading; distance=0 is silently ignored */
        uint16_t cm = 0;
        bool ok = lidar.readDistance(cm);
        if (ok && cm > 0) {
            if (cm < closestCm) {
                closestCm = cm;
            }
        }

        /* Check for sweep limit — commit displayCm and flip direction */
        if (servoAngle <= SWEEP_MIN_DEG || servoAngle >= SWEEP_MAX_DEG) {
            displayCm = closestCm;
            closestCm = DIST_MAX_CM;

            /* Print one line per half-sweep */
            int mood = distanceToMood(displayCm);
            Serial.print('[');
            Serial.print(moodLabel(mood));
            Serial.print(F("] Closest: "));
            Serial.print(displayCm);
            Serial.println(F(" cm"));

            /* Flip sweep direction */
            servoDir = -servoDir;
        }

        /* Advance angle for next step, clamped to sweep window */
        servoAngle += servoDir;
        if (servoAngle < SWEEP_MIN_DEG) servoAngle = SWEEP_MIN_DEG;
        if (servoAngle > SWEEP_MAX_DEG) servoAngle = SWEEP_MAX_DEG;
    }

    /* ----------------------------------------------------------
     * Tone sequencer — non-blocking, runs every loop iteration
     * ---------------------------------------------------------- */
    if (now >= noteEndMs) {
        /* Determine current mood from displayCm */
        int newMood = distanceToMood(displayCm);

        /* Mood change: restart phrase from the beginning */
        if (newMood != activeMood) {
            activeMood = newMood;
            noteIndex  = 0;
        }

        /* Look up current note */
        const Note *table  = moodTables[activeMood];
        uint8_t     size   = moodTableSizes[activeMood];
        uint16_t    freq   = table[noteIndex].freq;
        uint16_t    dur_ms = table[noteIndex].dur_ms;

        /* Play note or rest */
        if (freq == 0) {
            noTone(SPEAKER_PIN);
        } else {
            tone(SPEAKER_PIN, freq, dur_ms);
        }

        /* Schedule next note boundary */
        noteEndMs = now + (uint32_t)dur_ms + (uint32_t)GAP_MS;

        /* Advance note index, wrapping around the table */
        noteIndex = (noteIndex + 1) % size;
    }
}
