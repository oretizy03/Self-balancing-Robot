#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// USER SETTINGS
// ============================================================

// Choose which motor stays fixed at FIXED_PWM during calibration.
enum FixedMotor
{
  FIX_LEFT,
  FIX_RIGHT
};

const FixedMotor FIXED_MOTOR = FIX_RIGHT;   // <-- CHANGE THIS

const int FIXED_PWM = 200;

// Encoder settings
const float MOTOR_A_PULSES_PER_REV = 40.0f; // LEFT
const float MOTOR_B_PULSES_PER_REV = 40.0f; // RIGHT

// Use only one edge of the encoder signal.
// Change to 100-500 us if necessary for your encoder/noise level.
const unsigned long ENCODER_MIN_INTERVAL_US = 200;

// Calibration measurement
const unsigned long CAL_SAMPLE_TIME = 2000; // ms per measurement
const int CAL_AVERAGE_SAMPLES = 3;
const unsigned long CAL_SETTLE_TIME = 1000;  // ms after changing PWM

// Target speed matching tolerance.
const float CAL_TOLERANCE_PERCENT = 3.0f;
const int MATCH_CONFIRMATIONS = 4;

// PWM adjustment limits for the motor being calibrated.
const int CAL_MIN_PWM = 0;
const int CAL_MAX_PWM = 255;
const int CAL_MIN_STEP = 1;
const int CAL_MAX_STEP = 25;
const float CAL_GAIN = 0.255f;

const int channelA = 0;
const int channelB = 1;

// Safety limit: prevents calibration from endlessly pushing PWM.
const int CAL_MAX_CORRECTION = 150;

// ============================================================
// SPEED TEST SETTINGS
// ============================================================

const unsigned long SPEED_TEST_SETTLE_TIME = 500;
const unsigned long SPEED_TEST_TIME = 3000;
const int SPEED_TEST_SAMPLES = 3;
const float SPEED_TEST_MATCH_PERCENT = 2.5f;

struct SpeedMeasurement
{
  float ppsA;
  float ppsB;
  float rpmA;
  float rpmB;
};

// ============================================================
// BUTTON / BRAKE SETTINGS
// ============================================================

const unsigned long BUTTON_DEBOUNCE = 30;

bool activateBrakes = false;
const unsigned long BRAKE_TIME = 60;

// ============================================================
// PINS
// ============================================================

// Motor A = LEFT
#define PWMA 25
#define AIN1 26
#define AIN2 27

// Motor B = RIGHT
#define PWMB 14
#define BIN1 4
#define BIN2 13

#define STBY 33

#define MOTOR_A_ENCODER 18
#define MOTOR_B_ENCODER 19

#define BUZZER_PIN 23
#define SWITCH_PIN 35

// ============================================================
// GLOBALS
// ============================================================

volatile unsigned long motorACount = 0;
volatile unsigned long motorBCount = 0;

volatile unsigned long lastAEncoderMicros = 0;
volatile unsigned long lastBEncoderMicros = 0;

int motorAPWM = FIXED_PWM;
int motorBPWM = FIXED_PWM;

bool calibrationComplete = false;
bool buttonArmed = false;

float savedMotorAPPS = 0.0f;
float savedMotorBPPS = 0.0f;
float savedMotorARPM = 0.0f;
float savedMotorBRPM = 0.0f;

int savedMotorAPWM = FIXED_PWM;
int savedMotorBPWM = FIXED_PWM;

Preferences preferences;

// ============================================================
// ENCODER INTERRUPTS
// ============================================================

// IMPORTANT:
// We now use RISING instead of CHANGE so one pulse is counted once.
// The minimum-time filter rejects extremely fast false transitions.

void IRAM_ATTR motorAISR()
{
  unsigned long now = micros();

  if (now - lastAEncoderMicros >= ENCODER_MIN_INTERVAL_US)
  {
    motorACount++;
    lastAEncoderMicros = now;
  }
}

void IRAM_ATTR motorBISR()
{
  unsigned long now = micros();

  if (now - lastBEncoderMicros >= ENCODER_MIN_INTERVAL_US)
  {
    motorBCount++;
    lastBEncoderMicros = now;
  }
}

// ============================================================
// BASIC MOTOR FUNCTIONS
// ============================================================

void beepOnce()
{
  tone(BUZZER_PIN, 2000);
  delay(100);
  noTone(BUZZER_PIN);
}

void beepTwice()
{
  for (int i = 0; i < 2; i++)
  {
    tone(BUZZER_PIN, 2000);
    delay(100);
    noTone(BUZZER_PIN);

    if (i == 0)
      delay(100);
  }
}

void beepTriple()
{
  for (int i = 0; i < 3; i++)
  {
    tone(BUZZER_PIN, 2500);
    delay(100);
    noTone(BUZZER_PIN);

    if (i < 2)
      delay(100);
  }
}

void setForwardDirection()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void writeMotorPWM()
{
  motorAPWM = constrain(motorAPWM, 0, CAL_MAX_PWM);
  motorBPWM = constrain(motorBPWM, 0, CAL_MAX_PWM);

  ledcWrite(channelA, motorAPWM);
  ledcWrite(channelB, motorBPWM);
}

void stopMotors()
{
  ledcWrite(channelA, 0);
  ledcWrite(channelB, 0);
}

void brakeMotorA()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  ledcWrite(channelA, 255);
  delay(BRAKE_TIME);
  ledcWrite(channelA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void brakeMotorB()
{
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  ledcWrite(channelB, 255);
  delay(BRAKE_TIME);
  ledcWrite(channelB, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void stopAndOptionalBrake()
{
  stopMotors();

  if (activateBrakes)
  {
    brakeMotorA();
    brakeMotorB();
  }
}

// ============================================================
// ENCODER COUNT MANAGEMENT
// ============================================================

void resetAllEncoderCounts()
{
  unsigned long now = micros();

  noInterrupts();
  motorACount = 0;
  motorBCount = 0;
  lastAEncoderMicros = now;
  lastBEncoderMicros = now;
  interrupts();
}

void readAndClearBothCounts(unsigned long &countA, unsigned long &countB)
{
  noInterrupts();
  countA = motorACount;
  countB = motorBCount;
  motorACount = 0;
  motorBCount = 0;
  interrupts();
}

// ============================================================
// SPEED MEASUREMENT
// ============================================================

SpeedMeasurement measureSpeedOnce(unsigned long durationMs)
{
  SpeedMeasurement result;

  resetAllEncoderCounts();
  unsigned long startTime = millis();

  while (millis() - startTime < durationMs)
  {
    yield();
  }

  unsigned long countA = 0;
  unsigned long countB = 0;
  readAndClearBothCounts(countA, countB);

  float seconds = durationMs / 1000.0f;

  result.ppsA = countA / seconds;
  result.ppsB = countB / seconds;

  result.rpmA = (result.ppsA / MOTOR_A_PULSES_PER_REV) * 60.0f;
  result.rpmB = (result.ppsB / MOTOR_B_PULSES_PER_REV) * 60.0f;

  return result;
}

SpeedMeasurement measureAverageSpeed()
{
  SpeedMeasurement result = {0, 0, 0, 0};

  for (int i = 0; i < CAL_AVERAGE_SAMPLES; i++)
  {
    SpeedMeasurement sample = measureSpeedOnce(CAL_SAMPLE_TIME);

    result.ppsA += sample.ppsA;
    result.ppsB += sample.ppsB;
    result.rpmA += sample.rpmA;
    result.rpmB += sample.rpmB;
  }

  result.ppsA /= CAL_AVERAGE_SAMPLES;
  result.ppsB /= CAL_AVERAGE_SAMPLES;
  result.rpmA /= CAL_AVERAGE_SAMPLES;
  result.rpmB /= CAL_AVERAGE_SAMPLES;

  return result;
}

float speedDifferencePercent(float speedA, float speedB)
{
  float average = (speedA + speedB) / 2.0f;

  if (average <= 0.0f)
    return 100.0f;

  return (abs(speedA - speedB) / average) * 100.0f;
}

// ============================================================
// CALIBRATION STORAGE
// ============================================================

void saveCalibration(const SpeedMeasurement &speed)
{
  savedMotorAPWM = motorAPWM;
  savedMotorBPWM = motorBPWM;

  savedMotorAPPS = speed.ppsA;
  savedMotorBPPS = speed.ppsB;

  savedMotorARPM = speed.rpmA;
  savedMotorBRPM = speed.rpmB;

  preferences.begin("motorCal", false);

  preferences.putInt("motorAPWM", savedMotorAPWM);
  preferences.putInt("motorBPWM", savedMotorBPWM);
  preferences.putFloat("motorAPPS", savedMotorAPPS);
  preferences.putFloat("motorBPPS", savedMotorBPPS);
  preferences.putFloat("motorARPM", savedMotorARPM);
  preferences.putFloat("motorBRPM", savedMotorBRPM);
  preferences.putBool("valid", true);

  preferences.end();

  Serial.println();
  Serial.println("========================================");
  Serial.println("CALIBRATION SAVED");
  Serial.println("========================================");
  Serial.print("LEFT PWM  : ");
  Serial.println(savedMotorAPWM);
  Serial.print("RIGHT PWM : ");
  Serial.println(savedMotorBPWM);
  Serial.print("LEFT PPS  : ");
  Serial.println(savedMotorAPPS, 2);
  Serial.print("RIGHT PPS : ");
  Serial.println(savedMotorBPPS, 2);
  Serial.print("LEFT RPM  : ");
  Serial.println(savedMotorARPM, 2);
  Serial.print("RIGHT RPM : ");
  Serial.println(savedMotorBRPM, 2);
  Serial.print("MATCH ERR : ");
  Serial.print(speedDifferencePercent(savedMotorAPPS, savedMotorBPPS), 2);
  Serial.println(" %");
}

// ============================================================
// CALIBRATION
// ============================================================

void calibrateMotors()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("MOTOR CALIBRATION");
  Serial.println("========================================");

  if (FIXED_MOTOR == FIX_LEFT)
    Serial.println("REFERENCE: LEFT motor fixed");
  else
    Serial.println("REFERENCE: RIGHT motor fixed");

  Serial.print("FIXED PWM: ");
  Serial.println(FIXED_PWM);
  Serial.println("The other motor will be adjusted only.");
  Serial.println();

  calibrationComplete = false;

  setForwardDirection();
  delay(1000);

  // Start both motors at the fixed PWM.
  motorAPWM = FIXED_PWM;
  motorBPWM = FIXED_PWM;
  writeMotorPWM();

  delay(SPEED_TEST_SETTLE_TIME);

  int adjustablePWM = FIXED_PWM;
  int consecutiveMatches = 0;

  while (!calibrationComplete)
  {
    SpeedMeasurement speed = measureAverageSpeed();

    float matchError = speedDifferencePercent(speed.ppsA, speed.ppsB);

    Serial.println();
    Serial.println("--- Calibration measurement ---");
    Serial.print("LEFT  PWM: ");
    Serial.print(motorAPWM);
    Serial.print(" | ");
    Serial.print(speed.ppsA, 2);
    Serial.print(" PPS | ");
    Serial.print(speed.rpmA, 2);
    Serial.println(" RPM");

    Serial.print("RIGHT PWM: ");
    Serial.print(motorBPWM);
    Serial.print(" | ");
    Serial.print(speed.ppsB, 2);
    Serial.print(" PPS | ");
    Serial.print(speed.rpmB, 2);
    Serial.println(" RPM");

    Serial.print("Match error: ");
    Serial.print(matchError, 2);
    Serial.println(" %");

    if (speed.ppsA <= 0.0f || speed.ppsB <= 0.0f)
    {
      Serial.println("Encoder speed is zero. Check encoder wiring/mechanics.");
      consecutiveMatches = 0;
      delay(300);
      continue;
    }

    // Both are within the allowed speed difference.
    if (matchError <= CAL_TOLERANCE_PERCENT)
    {
      consecutiveMatches++;

      Serial.print("MATCH CONFIRMATION: ");
      Serial.print(consecutiveMatches);
      Serial.print("/");
      Serial.println(MATCH_CONFIRMATIONS);

      if (consecutiveMatches >= MATCH_CONFIRMATIONS)
      {
        calibrationComplete = true;

        saveCalibration(speed);
        stopAndOptionalBrake();

        Serial.println();
        Serial.println("*** CALIBRATION SUCCESSFUL ***");
        beepTriple();
        buttonArmed = false;
        return;
      }

      continue;
    }

    consecutiveMatches = 0;

    // Which motor needs to change?
    float fixedSpeed;
    float adjustableSpeed;

    if (FIXED_MOTOR == FIX_LEFT)
    {
      fixedSpeed = speed.ppsA;
      adjustableSpeed = speed.ppsB;
    }
    else
    {
      fixedSpeed = speed.ppsB;
      adjustableSpeed = speed.ppsA;
    }

    // Positive error means adjustable motor needs to slow down.
    // Negative error means adjustable motor needs to speed up.
    float relativeError = (adjustableSpeed - fixedSpeed) / fixedSpeed;

    int step = (int)round(abs(relativeError) * FIXED_PWM * CAL_GAIN);
    step = constrain(step, CAL_MIN_STEP, CAL_MAX_STEP);

    if (relativeError > 0.0f)
    {
      adjustablePWM -= step;
      Serial.print("Adjustable motor is FASTER -> PWM -");
      Serial.println(step);
    }
    else
    {
      adjustablePWM += step;
      Serial.print("Adjustable motor is SLOWER -> PWM +");
      Serial.println(step);
    }

    adjustablePWM = constrain(adjustablePWM,
                              max(CAL_MIN_PWM, FIXED_PWM - CAL_MAX_CORRECTION),
                              min(CAL_MAX_PWM, FIXED_PWM + CAL_MAX_CORRECTION));

    if (FIXED_MOTOR == FIX_LEFT)
    {
      motorAPWM = FIXED_PWM;
      motorBPWM = adjustablePWM;
    }
    else
    {
      motorAPWM = adjustablePWM;
      motorBPWM = FIXED_PWM;
    }

    writeMotorPWM();

    Serial.print("New LEFT PWM : ");
    Serial.println(motorAPWM);
    Serial.print("New RIGHT PWM: ");
    Serial.println(motorBPWM);

    beepOnce();
    delay(CAL_SETTLE_TIME);
  }
}

// ============================================================
// SPEED TEST BUTTON ACTION
// ============================================================

void speedMatchTest()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("MOTOR SPEED MATCH TEST");
  Serial.println("========================================");
  Serial.println("Both motors will run at their saved PWM.");
  Serial.println();

  stopMotors();
  setForwardDirection();
  delay(SPEED_TEST_SETTLE_TIME);

  motorAPWM = savedMotorAPWM;
  motorBPWM = savedMotorBPWM;
  writeMotorPWM();

  Serial.print("1 SECOND DELAY");
  Serial.print(" ");
  delay(1000);

  float totalPPSA = 0.0f;
  float totalPPSB = 0.0f;
  float totalRPMA = 0.0f;
  float totalRPMB = 0.0f;

  for (int i = 0; i < SPEED_TEST_SAMPLES; i++)
  {
    SpeedMeasurement speed = measureSpeedOnce(SPEED_TEST_TIME);

    totalPPSA += speed.ppsA;
    totalPPSB += speed.ppsB;
    totalRPMA += speed.rpmA;
    totalRPMB += speed.rpmB;

    Serial.print("Test ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.println(SPEED_TEST_SAMPLES);

    Serial.print("  LEFT : ");
    Serial.print(speed.ppsA, 2);
    Serial.print(" PPS | ");
    Serial.print(speed.rpmA, 2);
    Serial.println(" RPM");

    Serial.print("  RIGHT: ");
    Serial.print(speed.ppsB, 2);
    Serial.print(" PPS | ");
    Serial.print(speed.rpmB, 2);
    Serial.println(" RPM");

    Serial.print("  Error: ");
    Serial.print(speedDifferencePercent(speed.ppsA, speed.ppsB), 2);
    Serial.println(" %");
    Serial.println();
  }

  stopAndOptionalBrake();

  float avgPPSA = totalPPSA / SPEED_TEST_SAMPLES;
  float avgPPSB = totalPPSB / SPEED_TEST_SAMPLES;
  float avgRPMA = totalRPMA / SPEED_TEST_SAMPLES;
  float avgRPMB = totalRPMB / SPEED_TEST_SAMPLES;
  float finalError = speedDifferencePercent(avgPPSA, avgPPSB);

  Serial.println("========================================");
  Serial.println("FINAL SPEED TEST RESULT");
  Serial.println("========================================");

  Serial.print("LEFT PWM       : ");
  Serial.println(savedMotorAPWM);
  Serial.print("RIGHT PWM      : ");
  Serial.println(savedMotorBPWM);

  Serial.print("LEFT AVG PPS   : ");
  Serial.println(avgPPSA, 2);
  Serial.print("RIGHT AVG PPS  : ");
  Serial.println(avgPPSB, 2);

  Serial.print("LEFT AVG RPM   : ");
  Serial.println(avgRPMA, 2);
  Serial.print("RIGHT AVG RPM  : ");
  Serial.println(avgRPMB, 2);

  Serial.print("SPEED DIFFERENCE: ");
  Serial.print(finalError, 2);
  Serial.println(" %");

  if (finalError <= SPEED_TEST_MATCH_PERCENT)
  {
    Serial.println();
    Serial.println("RESULT: SPEEDS ARE SUCCESSFULLY MATCHED.");
    beepTriple();
  }
  else
  {
    Serial.println();
    Serial.println("RESULT: SPEEDS ARE NOT YET MATCHED.");
    Serial.println("Check encoder readings and repeat calibration.");
    beepOnce();
  }
}

// ============================================================
// BUTTON HANDLING
// ============================================================

void checkButton()
{
  if (!calibrationComplete)
    return;

  bool pressed = (digitalRead(SWITCH_PIN) == LOW);

  if (!buttonArmed)
  {
    if (!pressed)
    {
      buttonArmed = true;
      Serial.println("Button armed.");
    }

    return;
  }

  if (pressed)
  {
    delay(BUTTON_DEBOUNCE);

    if (digitalRead(SWITCH_PIN) == LOW)
    {
      buttonArmed = false;

      Serial.println();
      Serial.println("BUTTON PRESSED -> SPEED MATCH TEST");

      speedMatchTest();

      while (digitalRead(SWITCH_PIN) == LOW)
        delay(5);

      Serial.println("Button released.");
      Serial.println("System ready.");
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(MOTOR_A_ENCODER, INPUT_PULLUP);
  pinMode(MOTOR_B_ENCODER, INPUT_PULLUP);

  // GPIO 35 is input-only and has no internal pull-up.
  // Your external wiring is:
  // 3.3V -> 10k resistor -> GPIO35
  // switch -> GND
  pinMode(SWITCH_PIN, INPUT);

  // Arduino-ESP32 LEDC API used by your original sketch.
  ledcSetup(channelA, 1000, 8);
  ledcSetup(channelB, 1000, 8);

  // 3. Attach the physical pins to those channels
  ledcAttachPin(PWMA, channelA);
  ledcAttachPin(PWMB, channelB);

  digitalWrite(STBY, HIGH);

  setForwardDirection();
  stopMotors();
  resetAllEncoderCounts();

  attachInterrupt(
    digitalPinToInterrupt(MOTOR_A_ENCODER),
    motorAISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(MOTOR_B_ENCODER),
    motorBISR,
    RISING
  );

  beepTwice();

  calibrateMotors();

  Serial.println();
  Serial.println("========================================");
  Serial.println("SYSTEM READY");
  Serial.println("========================================");
  Serial.println("Press button for SPEED MATCH TEST.");
  Serial.print("Reference motor: ");
  Serial.println(FIXED_MOTOR == FIX_LEFT ? "LEFT" : "RIGHT");
  Serial.print("Reference PWM: ");
  Serial.println(FIXED_PWM);
  Serial.print("Brake system: ");
  Serial.println(activateBrakes ? "ENABLED" : "DISABLED");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  checkButton();
}
