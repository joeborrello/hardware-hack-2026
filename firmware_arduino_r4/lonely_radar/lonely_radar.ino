/* ============================================================
 * lonely_radar — LiDAR sweep + servo + mood buzzer
 * Target: Arduino Uno R4 (WiFi or Minima)
 *
 * Hardware wiring
 * ---------------
 * LIDAR-Lite v3 (same as pet_lidar):
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
 * Passive buzzer (or small speaker with series resistor ~100 Ω):
 *   Positive leg         -> D8
 *   Other leg            -> GND
 * ============================================================ */

#include <Wire.h>
#include <Servo.h>
#include "LidarLiteV3.h"

/* ---- Pin assignments ---- */
#define SERVO_PIN    9
#define SPEAKER_PIN  8

/* ---- Sweep parameters ---- */
#define SWEEP_STEP_DEG   2    /* degrees per step                  */
#define SWEEP_STEP_MS   10    /* delay between steps (ms)          */
#define SERVO_MIN_DEG    0
#define SERVO_MAX_DEG  180

/* ---- Mood distance thresholds (cm) ---- */
#define ZONE_CLOSE_CM   80   /* ≤ 80 cm  → EXCITED  */
#define ZONE_NEAR_CM   150   /* ≤ 150 cm → HAPPY    */
#define ZONE_FAR_CM    250   /* ≤ 250 cm → HOPEFUL  */
                             /* > 250 cm → LONELY   */

/* ---- Note frequencies (Hz) ---- */
#define NOTE_D4   294
#define NOTE_E4   330
#define NOTE_G4   392
#define NOTE_A4   440
#define NOTE_C5   523
#define NOTE_E5   659
#define NOTE_G5   784
#define NOTE_C6  1047

/* ---- Mood identifiers ---- */
#define MOOD_LONELY   0
#define MOOD_HOPEFUL  1
#define MOOD_HAPPY    2
#define MOOD_EXCITED  3

/* ---- Objects ---- */
LidarLiteV3 lidar;
Servo       servo;

/* ---- State ---- */
static int  currentMood = MOOD_LONELY;

/* ============================================================
 * Helper: play one note then wait (blocking, acceptable for
 * hackathon use — keeps the sketch simple and self-contained).
 * ============================================================ */
static void playNote(unsigned int freq, unsigned int durationMs, unsigned int gapMs)
{
    tone(SPEAKER_PIN, freq, durationMs);
    delay(durationMs + gapMs);
}

/* ============================================================
 * Mood phrases — each is a short blocking musical sequence.
 * ============================================================ */
static void playLonely(void)
{
    /* Slow, minor-key, descending — played twice */
    for (uint8_t rep = 0; rep < 2; rep++) {
        playNote(NOTE_A4, 500, 100);
        playNote(NOTE_G4, 500, 100);
        playNote(NOTE_E4, 500, 100);
        playNote(NOTE_D4, 500, 100);
    }
}

static void playHopeful(void)
{
    /* Slightly faster, minor but rising at the end — played once */
    playNote(NOTE_D4, 350, 80);
    playNote(NOTE_E4, 350, 80);
    playNote(NOTE_G4, 350, 80);
    playNote(NOTE_A4, 350, 80);
}

static void playHappy(void)
{
    /* Upbeat major arpeggio — played once */
    playNote(NOTE_C5, 200, 60);
    playNote(NOTE_E5, 200, 60);
    playNote(NOTE_G5, 200, 60);
    playNote(NOTE_C6, 200, 60);
}

static void playExcited(void)
{
    /* Fast, high, major arpeggio up and back — played once */
    playNote(NOTE_C5, 100, 30);
    playNote(NOTE_E5, 100, 30);
    playNote(NOTE_G5, 100, 30);
    playNote(NOTE_C6, 100, 30);
    playNote(NOTE_G5, 100, 30);
    playNote(NOTE_E5, 100, 30);
    playNote(NOTE_C5, 100, 30);
}

/* ============================================================
 * Dispatch to the correct phrase for the current mood.
 * ============================================================ */
static void playMood(int mood)
{
    switch (mood) {
        case MOOD_LONELY:  playLonely();  break;
        case MOOD_HOPEFUL: playHopeful(); break;
        case MOOD_HAPPY:   playHappy();   break;
        case MOOD_EXCITED: playExcited(); break;
        default:           break;
    }
}

/* ============================================================
 * Map a minimum distance to a mood.
 * ============================================================ */
static int distanceToMood(uint16_t minCm)
{
    if (minCm <= ZONE_CLOSE_CM) return MOOD_EXCITED;
    if (minCm <= ZONE_NEAR_CM)  return MOOD_HAPPY;
    if (minCm <= ZONE_FAR_CM)   return MOOD_HOPEFUL;
    return MOOD_LONELY;
}

/* ============================================================
 * Return a human-readable mood label.
 * ============================================================ */
static const char *moodLabel(int mood)
{
    switch (mood) {
        case MOOD_LONELY:  return "LONELY";
        case MOOD_HOPEFUL: return "HOPEFUL";
        case MOOD_HAPPY:   return "HAPPY";
        case MOOD_EXCITED: return "EXCITED";
        default:           return "UNKNOWN";
    }
}

/* ============================================================
 * Perform one sweep (0→180 or 180→0), updating minDist_cm.
 * Returns true if at least one valid reading was obtained.
 * ============================================================ */
static bool doSweep(int fromDeg, int toDeg, uint16_t &minDist_cm)
{
    bool gotReading = false;
    int  step = (toDeg > fromDeg) ? SWEEP_STEP_DEG : -SWEEP_STEP_DEG;

    for (int angle = fromDeg; ; angle += step) {
        /* Clamp to endpoint so we always land exactly on it */
        if (step > 0 && angle > toDeg) angle = toDeg;
        if (step < 0 && angle < toDeg) angle = toDeg;

        servo.write(angle);
        delay(SWEEP_STEP_MS);

        /* Fire a LiDAR reading; skip silently on error */
        uint16_t dist;
        if (lidar.readDistance(dist)) {
            if (!gotReading || dist < minDist_cm) {
                minDist_cm = dist;
            }
            gotReading = true;
        }

        if (angle == toDeg) break;
    }

    return gotReading;
}

/* ============================================================
 * setup
 * ============================================================ */
void setup()
{
    Serial.begin(115200);
    while (!Serial);   /* wait for USB CDC on R4 */

    Serial.println("lonely_radar — starting up");

    /* Attach servo and park at 0° */
    servo.attach(SERVO_PIN);
    servo.write(SERVO_MIN_DEG);
    delay(500);

    /* Initialise LiDAR */
    if (!lidar.begin()) {
        Serial.println("ERROR: LIDAR not found (I2C 0x62). Check wiring.");
        while (1) {}
    }
    Serial.println("LIDAR ready — beginning sweep");
}

/* ============================================================
 * loop — one full back-and-forth sweep per iteration
 * ============================================================ */
void loop()
{
    uint16_t minDist_cm = 0;
    bool     gotAny     = false;

    /* Forward sweep: 0° → 180° */
    bool fwd = doSweep(SERVO_MIN_DEG, SERVO_MAX_DEG, minDist_cm);
    gotAny = fwd;

    /* Reverse sweep: 180° → 0°, continuing to track minimum */
    bool rev = doSweep(SERVO_MAX_DEG, SERVO_MIN_DEG, minDist_cm);
    gotAny = gotAny || rev;

    /* Update mood only if we got at least one valid reading */
    if (gotAny) {
        currentMood = distanceToMood(minDist_cm);
    }

    /* Print one line per sweep */
    Serial.print("[");
    Serial.print(moodLabel(currentMood));
    Serial.print("] Closest: ");
    Serial.print(gotAny ? minDist_cm : 0);
    Serial.println(" cm");

    /* Play the mood phrase between sweeps */
    playMood(currentMood);
}
