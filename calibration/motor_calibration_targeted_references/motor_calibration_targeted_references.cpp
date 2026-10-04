#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// SPEED MEASUREMENT STRUCTURE
// Must be declared before any functions because of the Arduino
// IDE's automatic function-prototype generation.
// ============================================================

struct SpeedMeasurement
{
  float rpmA;
  float rpmB;
  float ppsA;
  float ppsB;
};

// ============================================================
// USER SETTINGS
// ============================================================

// Choose which motor is the fixed reference.
enum FixedMotor
{
  FIX_LEFT,
  FIX_RIGHT
};

// For your current project, use FIX_RIGHT.
const FixedMotor FIXED_MOTOR = FIX_RIGHT;

// This is the motor-speed operating point you actually care about.
const int FIXED_PWM = 200;

// Encoder counts per mechanical wheel revolution.
// Change these only after you have verified the value by repeated
// manual-rotation tests.
const float LEFT_PULSES_PER_REV  = 40.0f;
const float RIGHT_PULSES_PER_REV = 40.0f;

// ============================================================
// CALIBRATION SEARCH
// ============================================================

// Your measurements around RIGHT=200 repeatedly put LEFT near 140.
// The search therefore starts here rather than wasting time from 0.
const int SEARCH_CENTER_PWM = 140;

// The first coarse search checks center +/- this amount.
const int COARSE_OFFSET = 10;
const int COARSE_STEP = 5;

// After the coarse search, the best area is searched at 1-PWM resolution.
const int FINE_OFFSET = 5;
const int FINE_STEP = 1;

// PWM is 8-bit in this sketch, so the valid range is 0-255.
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// Measurement settings.
// Median filtering is used so one noisy encoder window does not
// dominate the result.
const unsigned long SETTLE_TIME_MS = 1000;
const unsigned long MEASUREMENT_WINDOW_MS = 700;
const int MEASUREMENT_SAMPLES = 5;

// Final acceptance requirement.
const float TARGET_MATCH_PERCENT = 3.5f;
const int FINAL_CONFIRM_SAMPLES = 5;

// Encoder glitch rejection.
// One rising edge is counted once, and edges closer together than
// this are ignored.
const unsigned long ENCODER_MIN_INTERVAL_US = 200;

// ============================================================
// SPEED TEST BUTTON SETTINGS
// ============================================================

const unsigned long SPEED_TEST_SETTLE_MS = 1000;
const unsigned long SPEED_TEST_WINDOW_MS = 1000;
const int SPEED_TEST_SAMPLES = 5;
const float SPEED_TEST_MATCH_PERCENT = 3.0f;

const unsigned long BUTTON_DEBOUNCE = 30;

// ============================================================
// OPTIONAL BRAKING
// ============================================================

bool activateBrakes = false;
const unsigned long BRAKE_TIME = 60;

// ============================================================
// PINS
// ============================================================

// LEFT motor
#define PWMA 25
#define AIN1 26
#define AIN2 27

// RIGHT motor
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

int savedMotorAPWM = FIXED_PWM;
int savedMotorBPWM = FIXED_PWM;

float savedMotorARPM = 0.0f;
float savedMotorBRPM = 0.0f;

float savedMotorAPPS = 0.0f;
float savedMotorBPPS = 0.0f;

bool calibrationComplete = false;
bool buttonArmed = false;

const int channelA = 0;
const int channelB = 1;

Preferences preferences;

// ============================================================
// ENCODER INTERRUPTS
// ============================================================

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
// BUZZER
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

// ============================================================
// MOTOR CONTROL
// ============================================================

void setForwardDirection()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void writeMotorPWM()
{
  motorAPWM = constrain(motorAPWM, MIN_PWM, MAX_PWM);
  motorBPWM = constrain(motorBPWM, MIN_PWM, MAX_PWM);

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
// ENCODER COUNT HANDLING
// ============================================================

void resetEncoderCounts()
{
  unsigned long now = micros();

  noInterrupts();
  motorACount = 0;
  motorBCount = 0;
  lastAEncoderMicros = now;
  lastBEncoderMicros = now;
  interrupts();
}

void readAndClearCounts(unsigned long &countA, unsigned long &countB)
{
  noInterrupts();
  countA = motorACount;
  countB = motorBCount;
  motorACount = 0;
  motorBCount = 0;
  interrupts();
}

// ============================================================
// ONE SPEED SAMPLE
// ============================================================

SpeedMeasurement measureSpeedOnce(unsigned long durationMs)
{
  SpeedMeasurement result = {0, 0, 0, 0};

  resetEncoderCounts();

  unsigned long startTime = millis();

  while (millis() - startTime < durationMs)
  {
    yield();
  }

  unsigned long countA;
  unsigned long countB;
  readAndClearCounts(countA, countB);

  float seconds = durationMs / 1000.0f;

  result.ppsA = countA / seconds;
  result.ppsB = countB / seconds;

  result.rpmA = (result.ppsA / LEFT_PULSES_PER_REV) * 60.0f;
  result.rpmB = (result.ppsB / RIGHT_PULSES_PER_REV) * 60.0f;

  return result;
}

// ============================================================
// MEDIAN HELPER
// ============================================================

float medianOf(float values[], int count)
{
  float sorted[10];

  if (count > 10)
    count = 10;

  for (int i = 0; i < count; i++)
    sorted[i] = values[i];

  for (int i = 0; i < count - 1; i++)
  {
    for (int j = i + 1; j < count; j++)
    {
      if (sorted[j] < sorted[i])
      {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  if (count % 2 == 1)
    return sorted[count / 2];

  return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
}

// ============================================================
// FILTERED SPEED MEASUREMENT
// ============================================================

SpeedMeasurement measureFilteredSpeed()
{
  SpeedMeasurement result = {0, 0, 0, 0};

  float rpmA[10];
  float rpmB[10];
  float ppsA[10];
  float ppsB[10];

  int sampleCount = MEASUREMENT_SAMPLES;

  if (sampleCount > 10)
    sampleCount = 10;

  for (int i = 0; i < sampleCount; i++)
  {
    SpeedMeasurement sample =
        measureSpeedOnce(MEASUREMENT_WINDOW_MS);

    rpmA[i] = sample.rpmA;
    rpmB[i] = sample.rpmB;
    ppsA[i] = sample.ppsA;
    ppsB[i] = sample.ppsB;
  }

  result.rpmA = medianOf(rpmA, sampleCount);
  result.rpmB = medianOf(rpmB, sampleCount);
  result.ppsA = medianOf(ppsA, sampleCount);
  result.ppsB = medianOf(ppsB, sampleCount);

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
// RUN A PWM CANDIDATE
// ============================================================

SpeedMeasurement testAdjustablePWM(int adjustablePWM)
{
  adjustablePWM = constrain(adjustablePWM, MIN_PWM, MAX_PWM);

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

  delay(SETTLE_TIME_MS);

  SpeedMeasurement speed = measureFilteredSpeed();

  Serial.print("PWM candidate = ");
  Serial.print(adjustablePWM);
  Serial.print(" | LEFT RPM = ");
  Serial.print(speed.rpmA, 2);
  Serial.print(" | RIGHT RPM = ");
  Serial.print(speed.rpmB, 2);
  Serial.print(" | Error = ");
  Serial.print(speedDifferencePercent(speed.rpmA, speed.rpmB), 2);
  Serial.println(" %");

  return speed;
}

// ============================================================
// CALIBRATION STORAGE
// ============================================================

void saveCalibration(const SpeedMeasurement &speed)
{
  savedMotorAPWM = motorAPWM;
  savedMotorBPWM = motorBPWM;

  savedMotorARPM = speed.rpmA;
  savedMotorBRPM = speed.rpmB;

  savedMotorAPPS = speed.ppsA;
  savedMotorBPPS = speed.ppsB;

  preferences.begin("motorCal", false);

  preferences.putInt("motorAPWM", savedMotorAPWM);
  preferences.putInt("motorBPWM", savedMotorBPWM);
  preferences.putFloat("motorARPM", savedMotorARPM);
  preferences.putFloat("motorBRPM", savedMotorBRPM);
  preferences.putFloat("motorAPPS", savedMotorAPPS);
  preferences.putFloat("motorBPPS", savedMotorBPPS);
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
  Serial.print("LEFT RPM  : ");
  Serial.println(savedMotorARPM, 2);
  Serial.print("RIGHT RPM : ");
  Serial.println(savedMotorBRPM, 2);
  Serial.print("MATCH ERR : ");
  Serial.print(speedDifferencePercent(savedMotorARPM, savedMotorBRPM), 2);
  Serial.println(" %");
}

// ============================================================
// DIRECT TARGET-POINT CALIBRATION
// ============================================================

void calibrateMotors()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("TARGET-POINT MOTOR CALIBRATION");
  Serial.println("========================================");

  if (FIXED_MOTOR == FIX_LEFT)
    Serial.println("REFERENCE: LEFT MOTOR");
  else
    Serial.println("REFERENCE: RIGHT MOTOR");

  Serial.print("REFERENCE PWM: ");
  Serial.println(FIXED_PWM);
  Serial.println();
  Serial.println("The reference motor will NOT change.");
  Serial.println("Only the other motor is searched.");
  Serial.println();

  calibrationComplete = false;

  setForwardDirection();

  // Start at the search centre.
  int bestPWM = constrain(SEARCH_CENTER_PWM, MIN_PWM, MAX_PWM);
  float bestError = 1000000.0f;
  SpeedMeasurement bestSpeed = {0, 0, 0, 0};

  // ----------------------------------------------------------
  // COARSE SEARCH
  // ----------------------------------------------------------

  Serial.println("--- COARSE SEARCH ---");
  delay(500);
  beepOnce();

  int coarseStart = SEARCH_CENTER_PWM - COARSE_OFFSET;
  int coarseEnd   = SEARCH_CENTER_PWM + COARSE_OFFSET;

  for (int pwm = coarseStart; pwm <= coarseEnd; pwm += COARSE_STEP)
  {
    if (pwm < MIN_PWM || pwm > MAX_PWM)
      continue;

    SpeedMeasurement speed = testAdjustablePWM(pwm);
    float error = speedDifferencePercent(speed.rpmA, speed.rpmB);

    if (error < bestError)
    {
      bestError = error;
      bestPWM = pwm;
      bestSpeed = speed;
    }
  }

  Serial.println();
  Serial.print("Best coarse PWM: ");
  Serial.print(bestPWM);
  Serial.print(" | Error: ");
  Serial.print(bestError, 2);
  Serial.println(" %");

  // ----------------------------------------------------------
  // FINE SEARCH
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("--- FINE SEARCH ---");

  beepOnce();

  int fineStart = bestPWM - FINE_OFFSET;
  int fineEnd   = bestPWM + FINE_OFFSET;

  for (int pwm = fineStart; pwm <= fineEnd; pwm += FINE_STEP)
  {
    if (pwm < MIN_PWM || pwm > MAX_PWM)
      continue;

    SpeedMeasurement speed = testAdjustablePWM(pwm);
    float error = speedDifferencePercent(speed.rpmA, speed.rpmB);

    if (error < bestError)
    {
      bestError = error;
      bestPWM = pwm;
      bestSpeed = speed;
    }
  }

  // ----------------------------------------------------------
  // APPLY BEST RESULT
  // ----------------------------------------------------------

  if (FIXED_MOTOR == FIX_LEFT)
  {
    motorAPWM = FIXED_PWM;
    motorBPWM = bestPWM;
  }
  else
  {
    motorAPWM = bestPWM;
    motorBPWM = FIXED_PWM;
  }

  writeMotorPWM();

  Serial.println();
  Serial.println("========================================");
  Serial.println("BEST PWM FOUND");
  Serial.println("========================================");
  Serial.print("LEFT PWM  : ");
  Serial.println(motorAPWM);
  Serial.print("RIGHT PWM : ");
  Serial.println(motorBPWM);

  delay(SETTLE_TIME_MS);

  // ----------------------------------------------------------
  // FINAL CONFIRMATION
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("--- FINAL CONFIRMATION ---");

  beepOnce();

  float finalRPM_A[10];
  float finalRPM_B[10];
  float finalError[10];

  int confirmations = FINAL_CONFIRM_SAMPLES;
  if (confirmations > 10)
    confirmations = 10;

  for (int i = 0; i < confirmations; i++)
  {
    SpeedMeasurement speed = measureSpeedOnce(1000);

    finalRPM_A[i] = speed.rpmA;
    finalRPM_B[i] = speed.rpmB;
    finalError[i] = speedDifferencePercent(speed.rpmA, speed.rpmB);

    Serial.print("Confirmation ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(confirmations);
    Serial.print(" | L = ");
    Serial.print(speed.rpmA, 2);
    Serial.print(" RPM | R = ");
    Serial.print(speed.rpmB, 2);
    Serial.print(" RPM | Error = ");
    Serial.print(finalError[i], 2);
    Serial.println(" %");
  }

  float confirmedRPM_A = medianOf(finalRPM_A, confirmations);
  float confirmedRPM_B = medianOf(finalRPM_B, confirmations);
  float confirmedError = speedDifferencePercent(confirmedRPM_A, confirmedRPM_B);

  Serial.println();
  Serial.println("========================================");
  Serial.println("FINAL CALIBRATION RESULT");
  Serial.println("========================================");
  Serial.print("LEFT PWM       : ");
  Serial.println(motorAPWM);
  Serial.print("RIGHT PWM      : ");
  Serial.println(motorBPWM);
  Serial.print("LEFT MEDIAN RPM: ");
  Serial.println(confirmedRPM_A, 2);
  Serial.print("RIGHT MEDIAN RPM: ");
  Serial.println(confirmedRPM_B, 2);
  Serial.print("MATCH ERROR    : ");
  Serial.print(confirmedError, 2);
  Serial.println(" %");

  if (confirmedError <= TARGET_MATCH_PERCENT)
  {
    calibrationComplete = true;

    SpeedMeasurement confirmedSpeed;
    confirmedSpeed.rpmA = confirmedRPM_A;
    confirmedSpeed.rpmB = confirmedRPM_B;
    confirmedSpeed.ppsA = (confirmedRPM_A / 60.0f) * LEFT_PULSES_PER_REV;
    confirmedSpeed.ppsB = (confirmedRPM_B / 60.0f) * RIGHT_PULSES_PER_REV;

    saveCalibration(confirmedSpeed);

    stopAndOptionalBrake();

    Serial.println();
    Serial.println("*** CALIBRATION SUCCESSFUL ***");
    beepTriple();
    Serial.println();
    Serial.println("========================================");
    Serial.println("SYSTEM READY");
    Serial.println("========================================");
    Serial.println("Press button for saved-PWM speed test.");
  }
  else
  {
    calibrationComplete = false;

    stopAndOptionalBrake();

    Serial.println();
    Serial.println("*** CALIBRATION DID NOT REACH TARGET ***");
    Serial.println("The closest measured PWM has been displayed but NOT saved.");
    Serial.println("Check encoder stability, mechanics and motor supply.");
    beepOnce();
  }
}

// ============================================================
// BUTTON SPEED TEST
// ============================================================

void speedMatchTest()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("SAVED-PWM SPEED MATCH TEST");
  Serial.println("========================================");
  Serial.println("Both motors run using the saved calibration.");
  Serial.println();

  setForwardDirection();

  motorAPWM = savedMotorAPWM;
  motorBPWM = savedMotorBPWM;
  writeMotorPWM();

  delay(SPEED_TEST_SETTLE_MS);

  float rpmA[10];
  float rpmB[10];
  float ppsA[10];
  float ppsB[10];

  int samples = SPEED_TEST_SAMPLES;
  if (samples > 10)
    samples = 10;

  for (int i = 0; i < samples; i++)
  {
    SpeedMeasurement speed = measureSpeedOnce(SPEED_TEST_WINDOW_MS);

    rpmA[i] = speed.rpmA;
    rpmB[i] = speed.rpmB;
    ppsA[i] = speed.ppsA;
    ppsB[i] = speed.ppsB;

    Serial.print("Test ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(samples);
    Serial.print(" | LEFT = ");
    Serial.print(speed.rpmA, 2);
    Serial.print(" RPM | RIGHT = ");
    Serial.print(speed.rpmB, 2);
    Serial.print(" RPM | Error = ");
    Serial.print(speedDifferencePercent(speed.rpmA, speed.rpmB), 2);
    Serial.println(" %");
  }

  float medianRPM_A = medianOf(rpmA, samples);
  float medianRPM_B = medianOf(rpmB, samples);
  float medianPPS_A = medianOf(ppsA, samples);
  float medianPPS_B = medianOf(ppsB, samples);

  float finalError = speedDifferencePercent(medianRPM_A, medianRPM_B);

  stopAndOptionalBrake();

  Serial.println();
  Serial.println("========================================");
  Serial.println("SPEED TEST RESULT");
  Serial.println("========================================");
  Serial.print("LEFT PWM       : ");
  Serial.println(savedMotorAPWM);
  Serial.print("RIGHT PWM      : ");
  Serial.println(savedMotorBPWM);
  Serial.print("LEFT MEDIAN RPM: ");
  Serial.println(medianRPM_A, 2);
  Serial.print("RIGHT MEDIAN RPM: ");
  Serial.println(medianRPM_B, 2);
  Serial.print("LEFT MEDIAN PPS: ");
  Serial.println(medianPPS_A, 2);
  Serial.print("RIGHT MEDIAN PPS: ");
  Serial.println(medianPPS_B, 2);
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
    Serial.println("RESULT: SPEEDS ARE NOT WITHIN THE TEST LIMIT.");
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
      Serial.println("BUTTON PRESSED -> SPEED TEST");

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

  // GPIO35 is input-only. External wiring:
  // 3.3V -> 10k resistor -> GPIO35
  // switch -> GND
  pinMode(SWITCH_PIN, INPUT);

  ledcSetup(channelA, 1000, 8);
  ledcSetup(channelB, 1000, 8);

  ledcAttachPin(PWMA, channelA);
  ledcAttachPin(PWMB, channelB);

  digitalWrite(STBY, HIGH);

  setForwardDirection();
  stopMotors();
  resetEncoderCounts();

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

}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  checkButton();
}
