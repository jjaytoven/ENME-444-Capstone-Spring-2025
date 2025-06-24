#include <math.h>     // fabs()

// ────────────────────────────────────────────────────────────────
//  USER-ADJUSTABLE DEFINES
// ────────────────────────────────────────────────────────────────
#define LED_ON   LOW      // for common-anode → LOW turns channel ON
#define LED_OFF  HIGH     //            "
/*  for common-cathode change to:
#define LED_ON   HIGH
#define LED_OFF  LOW
*/

// ── RGB LED pins (non-PWM digital pins are fine) ───────────────
const int redPin   = 28;
const int greenPin = 30;
const int bluePin  = 32;

// ── Push-button pin ─────────────────────────────────────────────
const int buttonPin = 3;

// ── Paddle hardware ─────────────────────────────────────────────
#define DIRECTION_PIN 10
#define PWM_PIN       11
#define ENCODER_PIN    2      // interrupt-capable
#define POT_PIN       A3      // pot between 3 V3 and GND

// ── Calibration θ = a + bV + cV² (deg) ─────────────────────────
const float cal_a = 43.56754;
const float cal_b = 55.93129;
const float cal_c = -72.45525;

// ── Encoder / geometry ─────────────────────────────────────────
const int   PULSES_PER_PADDLE_REV = 2160;
const float PULSES_PER_DEGREE     = PULSES_PER_PADDLE_REV / 360.0f;

// ── Motor constants ────────────────────────────────────────────
const int   HOLD_PWM   = 255;   // brake (verify for your driver)
const int   MOVE_PWM   = 10;    // gentle return
const float deadbandAngle = 2.0;   // ±2° neutral zone

// ── Notch positions for Case 2 ─────────────────────────────────
static const float notches[] = { -45, -30, -15, 0, 15, 30, 45 };
const int   NUM_NOTCHES = sizeof(notches)/sizeof(notches[0]);

// ── State vars ─────────────────────────────────────────────────
volatile long encoderPulses = 0;
bool  returning    = false;
long  targetPulses = 0;

int   buttonState     = 0;
int   lastButtonState = 0;
int   pressCount      = 0;      // mode = pressCount % 3

// ── ISR: count encoder pulses ──────────────────────────────────
void encoderISR() { encoderPulses++; }

// ── Helper: set LED colour on common-anode/cathode pin logic ───
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

// ── SETUP ──────────────────────────────────────────────────────
void setup()
{
  // I/O
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);
  pinMode(buttonPin, INPUT);

  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PWM_PIN,       OUTPUT);
  pinMode(ENCODER_PIN,   INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  // start braked, mode 0 (LED red)
  digitalWrite(DIRECTION_PIN, HIGH);
  analogWrite(PWM_PIN, HOLD_PWM);

  Serial.begin(115200);
}

// ── LOOP ───────────────────────────────────────────────────────
void loop()
{
  // 1) Push-button edge detect + debounce
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {           // rising edge
      pressCount++;                      // cycle modes 0→1→2→0…
      returning = false;                 // cancel any in-flight return
      analogWrite(PWM_PIN, HOLD_PWM);
      Serial.print(F("Mode → ")); Serial.println(pressCount % 3);
    }
    delay(50);
  }
  lastButtonState = buttonState;

  int mode = pressCount % 3;

  // 2) Mode-specific behaviour
  switch (mode) {

    // ───────────────────────────────────────────────────────
    case 0:   // SPRING RETURN (LED red)
    // ───────────────────────────────────────────────────────
      setColor(255, 0, 0);   // red
      // … (unchanged code from your original Case 0) …
      // (holds paddle, reads angle, returns to 0 if outside deadband)
      // ───────────────────────────────────────────────────────
      { // keep your existing Case 0 code block here
        if (!returning) {
          digitalWrite(DIRECTION_PIN, HIGH);
          analogWrite(PWM_PIN, HOLD_PWM);
        }
        int raw = analogRead(POT_PIN);
        float volt = (raw / 1023.0f) * 3.3f;
        float angle = cal_a + cal_b*volt + cal_c*volt*volt;
        if (!returning && fabs(angle) > deadbandAngle) {
          float excessDeg = fabs(angle) - deadbandAngle;
          targetPulses = (long)(excessDeg * PULSES_PER_DEGREE + 0.5f);
          noInterrupts(); encoderPulses = 0; interrupts();
          returning = true;
          digitalWrite(DIRECTION_PIN, angle > 0 ? LOW : HIGH);
          delay(500);
          analogWrite(PWM_PIN, MOVE_PWM);
        }
        if (returning && encoderPulses >= targetPulses) {
          analogWrite(PWM_PIN, HOLD_PWM);
          returning = false;
          Serial.println(F("↺ Returned into deadband"));
        }
        Serial.print(F("θ=")); Serial.print(angle,1);
        Serial.print(F("°, pulses=")); Serial.print(encoderPulses);
        Serial.print(F("/")); Serial.println(targetPulses);
      }
      break;

    // ───────────────────────────────────────────────────────
    case 1:   // IDLE BRAKE (LED green)
    // ───────────────────────────────────────────────────────
      setColor(0, 255, 0);   // green
      returning = false;
      analogWrite(PWM_PIN, HOLD_PWM);
      break;

    // ───────────────────────────────────────────────────────
    case 2:   // NOTCH RETURN (LED blue)
    // ───────────────────────────────────────────────────────
      setColor(0, 0, 255);   // blue

      // Hold when idle
      if (!returning) {
        digitalWrite(DIRECTION_PIN, HIGH);
        analogWrite(PWM_PIN, HOLD_PWM);
      }

      // Read pot → angle
      {
        int raw = analogRead(POT_PIN);
        float volt = (raw / 1023.0f) * 3.3f;
        float angle = cal_a + cal_b*volt + cal_c*volt*volt;

        // Find nearest notch
        float bestDiff = 1e6;
        float targetAngle = 0;
        for (int i = 0; i < NUM_NOTCHES; ++i) {
          float d = fabs(angle - notches[i]);
          if (d < bestDiff) {
            bestDiff = d;
            targetAngle = notches[i];
          }
        }

        // Start return to notch?
        if (!returning && bestDiff > deadbandAngle) {
          float excessDeg = bestDiff - deadbandAngle;
          targetPulses = (long)(excessDeg * PULSES_PER_DEGREE + 0.5f);
          noInterrupts(); encoderPulses = 0; interrupts();
          returning = true;
          // if current angle above target notch, drive negative to come back, else positive
          digitalWrite(DIRECTION_PIN, angle > targetAngle ? LOW : HIGH);
          delay(500);
          analogWrite(PWM_PIN, 1);
          break;  // skip finish check this pass
        }

        // Finish return?
        if (returning && encoderPulses >= targetPulses) {
          analogWrite(PWM_PIN, HOLD_PWM);
          returning = false;
          Serial.print(F("↺ Returned to notch "));
          Serial.println(targetAngle);
        }

        // Debug
        Serial.print(F("θ=")); Serial.print(angle,1);
        Serial.print(F("°, nearest notch=")); Serial.print(targetAngle,1);
        Serial.print(F("°, pulses=")); Serial.print(encoderPulses);
        Serial.print(F("/")); Serial.println(targetPulses);
      }
      break;
  }

  delay(50);   // loop pace
}
