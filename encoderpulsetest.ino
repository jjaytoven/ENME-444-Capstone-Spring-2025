//This is the final interation of Code - Jaden E. Dunning


// --- Pin definitions ---
#define DIRECTION_PIN 10
#define PWM_PIN       11
#define ENCODER_PIN    2    // Must be interrupt-capable
#define POT_PIN       A3    // Pot wiper between 3.3 V and GND

// --- Calibration (θ in degrees as function of V) ---
const float cal_a = 43.56754;
const float cal_b = 55.93129;
const float cal_c = -72.45525;

// --- Encoder/Paddle geometry ---
const int   PULSES_PER_PADDLE_REV = 2160;           // 6 pulses/motor-rev × 45:1 gearbox × 8:1 belt
const float PULSES_PER_DEGREE      = PULSES_PER_PADDLE_REV / 360.0;  // ≈6.0 pulses/deg

// --- Motor control speeds ---
const int HOLD_PWM   = 255;   // brake (stationary)
const int MOVE_PWM   = 10;    // gentle return speed

// --- Deadband around zero ---
const float deadbandAngle = 2.0;  // ±2° neutral zone

// --- State ---
volatile long encoderPulses = 0;
bool          returning    = false;  // are we in the “return” phase?
float         targetPulses = 0;      // pulses needed to return into deadband

// encoder ISR
void encoderISR() {
  encoderPulses++;
}

void setup() {
  Serial.begin(115200);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  // initially brake (hold center)
  digitalWrite(DIRECTION_PIN, HIGH);
  analogWrite(PWM_PIN, HOLD_PWM);
}

void loop() {
  // 1) Hold when idle


  if (!returning) {
    digitalWrite(DIRECTION_PIN, HIGH);
    analogWrite(PWM_PIN, HOLD_PWM);
  }

  // 2) Read pot & compute angle
  int raw     = analogRead(POT_PIN);
  float volt  = (raw / 1023.0) * 3.3;
  float angle = cal_a
              + cal_b * volt
              + cal_c * volt * volt;

  // 3) If moved outside deadband and not already returning → start return
  if (!returning && fabs(angle) > deadbandAngle) {
    // how far past the deadband?
    float excessDeg = fabs(angle) - deadbandAngle;
    // convert to pulses

    targetPulses = excessDeg * PULSES_PER_DEGREE;
    encoderPulses = 0;
    returning = true;

    // pick direction: angle>0 → push back → dir=LOW ; angle<0 → pull back → dir=HIGH
    if (angle > 0)      digitalWrite(DIRECTION_PIN, LOW);
    else /* angle<0 */  digitalWrite(DIRECTION_PIN, HIGH);
    delay(250);
    analogWrite(PWM_PIN, MOVE_PWM);
    // skip the rest so we don't re-brake this cycle
    return;
  }

  // 4) If we’re returning and have reached our pulse target → brake and finish
  if (returning && encoderPulses >= targetPulses) {
    analogWrite(PWM_PIN, HOLD_PWM);
    returning = false;
    Serial.println("↺ Returned into deadband.");
  }

  // 5) Debug output
  Serial.print("V=");    Serial.print(volt,3);
  Serial.print("  θ=");  Serial.print(angle,1);
  Serial.print("°  pulses="); Serial.print(encoderPulses);
  Serial.print("/");     Serial.print(targetPulses,1);
  Serial.print("  DB=±");Serial.print(deadbandAngle,1);
  Serial.println("°");

  delay(50);
}
