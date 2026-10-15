/*
 * MOTOR CALIBRATION (Stage 3 of 3): Target-point motor speed calibration
 
 * WHAT IT DOES
    The right motor is held at FIXED_PWM. The code tries different PWM values on the left motor (coarse steps around SEARCH_CENTER_PWM, then fine steps of 1) and reads both encoders each time to compare RPM. It picks the PWM where the two speeds are closest.
    It then re-checks that value five times. If the speeds match within 3.5%, it saves the PWM values to flash and beeps three times. If not, it saves nothing and beeps once.
    After a successful calibration, pressing the button runs both motors at the saved PWMs and reports whether they still match. summarize this

 * HOW TO USE
    1. Lift the wheels off the ground before powering the motors.
    2. On boot: 2 beeps, then calibration starts automatically.
       Reference motor stays at FIXED_PWM; only the other motor is searched.
       1 beep at the start of coarse search, fine search and confirmation.
    3. 3 beeps = matched within TARGET_MATCH_PERCENT and saved to flash.
       1 beep at the end = target missed, nothing saved.
    4. After success, press the button to run the saved-PWM speed test
       (3 beeps = matched, 1 beep = not matched).
    5. To recalibrate, press the ESP32 EN/reset button.

 * TUNING
    FIXED_MOTOR / FIXED_PWM       reference motor and its PWM
    SEARCH_CENTER_PWM             where the search starts (centre of range)
    TARGET_MATCH_PERCENT          acceptance limit for saving
    LEFT/RIGHT_PULSES_PER_REV     encoder counts per wheel revolution
 */

#include <Arduino.h>
#include <Preferences.h>

// SETTINGS - These are the only settings you should need to change for the robot.
enum FixedMotor { FIX_LEFT, FIX_RIGHT };
const FixedMotor FIXED_MOTOR = FIX_RIGHT;
const int FIXED_PWM = 200;
const int SEARCH_CENTER_PWM = 140;
const float LEFT_PULSES_PER_REV  = 40.0f;
const float RIGHT_PULSES_PER_REV = 40.0f;
const int COARSE_OFFSET = 10;
const int COARSE_STEP = 5;
const int FINE_OFFSET = 5;
const int FINE_STEP = 1;
const int MIN_PWM = 0;
const int MAX_PWM = 255;
const unsigned long SETTLE_TIME_MS = 1000;
const unsigned long MEASUREMENT_WINDOW_MS = 700;
const int MEASUREMENT_SAMPLES = 5;
const float TARGET_MATCH_PERCENT = 3.5f;
const int FINAL_CONFIRM_SAMPLES = 5;
const unsigned long CONFIRM_WINDOW_MS = 1000;
const unsigned long ENCODER_MIN_INTERVAL_US = 200;
const unsigned long SPEED_TEST_SETTLE_MS = 1000;
const unsigned long SPEED_TEST_WINDOW_MS = 1000;
const int SPEED_TEST_SAMPLES = 5;
const float SPEED_TEST_MATCH_PERCENT = 3.0f;
const unsigned long BUTTON_DEBOUNCE = 30;
const bool activateBrakes = false;
const unsigned long BRAKE_TIME = 60;
const int MAX_SAMPLES = 10;

// PINS
// LEFT motor
#define PWMA 25
#define AIN1 26
#define AIN2 27

// RIGHT motor (B)
#define PWMB 14
#define BIN1 4
#define BIN2 13

#define STBY 33
#define MOTOR_A_ENCODER 18
#define MOTOR_B_ENCODER 19
#define BUZZER_PIN 23
#define SWITCH_PIN 35

// GLOBALS
struct SpeedMeasurement
{
  float rpmA;
  float rpmB;
  float ppsA;
  float ppsB;
};

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

Preferences preferences;

const int PWM_FREQ = 1000;
const int PWM_RES_BITS = 8;

#if ESP_ARDUINO_VERSION_MAJOR >= 3

void pwmInit()
{
  ledcAttach(PWMA, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(BUZZER_PIN, 2000, PWM_RES_BITS);
}

void pwmWriteA(int duty) { ledcWrite(PWMA, duty); }
void pwmWriteB(int duty) { ledcWrite(PWMB, duty); }
void buzzerTone(unsigned int freq) { ledcWriteTone(BUZZER_PIN, freq); }

#else

const int CHANNEL_A = 0;
const int CHANNEL_B = 1;
const int CHANNEL_BUZZER = 2;

void pwmInit()
{
  ledcSetup(CHANNEL_A, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CHANNEL_B, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CHANNEL_BUZZER, 2000, PWM_RES_BITS);

  ledcAttachPin(PWMA, CHANNEL_A);
  ledcAttachPin(PWMB, CHANNEL_B);
  ledcAttachPin(BUZZER_PIN, CHANNEL_BUZZER);
}

void pwmWriteA(int duty) { ledcWrite(CHANNEL_A, duty); }
void pwmWriteB(int duty) { ledcWrite(CHANNEL_B, duty); }
void buzzerTone(unsigned int freq) { ledcWriteTone(CHANNEL_BUZZER, freq); }

#endif

void beep(int times, unsigned int freq = 2000)
{
  for (int i = 0; i < times; i++)
  {
    buzzerTone(freq);
    delay(100);
    buzzerTone(0);

    if (i < times - 1)
      delay(100);
  }
}

// ENCODER INTERRUPTS
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

// MOTOR CONTROL
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
  pwmWriteA(motorAPWM);
  pwmWriteB(motorBPWM);
}

void stopMotors()
{
  pwmWriteA(0);
  pwmWriteB(0);
}

void brakeMotors()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  pwmWriteA(255);
  pwmWriteB(255);

  delay(BRAKE_TIME);

  stopMotors();
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void stopAndOptionalBrake()
{
  stopMotors();
  if (activateBrakes)
    brakeMotors();
}

// MEASUREMENT
SpeedMeasurement measureSpeedOnce(unsigned long durationMs)
{
  resetEncoderCounts();
  unsigned long startUs = micros();
  delay(durationMs);
  unsigned long elapsedUs = micros() - startUs;
  unsigned long countA, countB;
  readAndClearCounts(countA, countB);
  float seconds = elapsedUs / 1000000.0f;
  SpeedMeasurement result;
  result.ppsA = countA / seconds;
  result.ppsB = countB / seconds;
  result.rpmA = (result.ppsA / LEFT_PULSES_PER_REV) * 60.0f;
  result.rpmB = (result.ppsB / RIGHT_PULSES_PER_REV) * 60.0f;
  return result;
}

float medianOf(const float values[], int count)
{
  float sorted[MAX_SAMPLES];
  for (int i = 0; i < count; i++)
    sorted[i] = values[i];
  for (int i = 1; i < count; i++)
  {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key)
    {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  if (count % 2 == 1)
    return sorted[count / 2];

  return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
}

SpeedMeasurement medianSpeed(const SpeedMeasurement samples[], int count)
{
  float rpmA[MAX_SAMPLES], rpmB[MAX_SAMPLES];
  float ppsA[MAX_SAMPLES], ppsB[MAX_SAMPLES];

  for (int i = 0; i < count; i++)
  {
    rpmA[i] = samples[i].rpmA;
    rpmB[i] = samples[i].rpmB;
    ppsA[i] = samples[i].ppsA;
    ppsB[i] = samples[i].ppsB;
  }

  SpeedMeasurement result;
  result.rpmA = medianOf(rpmA, count);
  result.rpmB = medianOf(rpmB, count);
  result.ppsA = medianOf(ppsA, count);
  result.ppsB = medianOf(ppsB, count);

  return result;
}

SpeedMeasurement measureFilteredSpeed()
{
  int count = constrain(MEASUREMENT_SAMPLES, 1, MAX_SAMPLES);
  SpeedMeasurement samples[MAX_SAMPLES];

  for (int i = 0; i < count; i++)
    samples[i] = measureSpeedOnce(MEASUREMENT_WINDOW_MS);

  return medianSpeed(samples, count);
}

float speedDifferencePercent(float speedA, float speedB)
{
  float average = (speedA + speedB) / 2.0f;

  if (average <= 0.0f)
    return 100.0f;

  return (fabsf(speedA - speedB) / average) * 100.0f;
}

// PRINT HELPERS
void printBanner(const char *title)
{
  Serial.println();
  Serial.println(" ");
  Serial.println(title);
  Serial.println(" ");
}

// CALIBRATION
void applyAdjustablePWM(int adjustablePWM)
{
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
}

SpeedMeasurement testAdjustablePWM(int adjustablePWM)
{
  adjustablePWM = constrain(adjustablePWM, MIN_PWM, MAX_PWM);

  applyAdjustablePWM(adjustablePWM);
  delay(SETTLE_TIME_MS);

  SpeedMeasurement speed = measureFilteredSpeed();

  Serial.printf("PWM %3d | LEFT %.2f RPM | RIGHT %.2f RPM | Error %.2f %%\n", adjustablePWM, speed.rpmA, speed.rpmB, speedDifferencePercent(speed.rpmA, speed.rpmB));

  return speed;
}

void searchRange(int start, int end, int step, int &bestPWM, float &bestError)
{
  for (int pwm = start; pwm <= end; pwm += step)
  {
    if (pwm < MIN_PWM || pwm > MAX_PWM)
      continue;

    SpeedMeasurement speed = testAdjustablePWM(pwm);
    float error = speedDifferencePercent(speed.rpmA, speed.rpmB);

    if (error < bestError)
    {
      bestError = error;
      bestPWM = pwm;
    }
  }
}

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

  printBanner("CALIBRATION SAVED");
  Serial.printf("LEFT PWM  : %d\n", savedMotorAPWM);
  Serial.printf("RIGHT PWM : %d\n", savedMotorBPWM);
  Serial.printf("LEFT RPM  : %.2f\n", savedMotorARPM);
  Serial.printf("RIGHT RPM : %.2f\n", savedMotorBRPM);
  Serial.printf("MATCH ERR : %.2f %%\n", speedDifferencePercent(savedMotorARPM, savedMotorBRPM));
}

void calibrateMotors()
{
  printBanner("TARGET-POINT MOTOR CALIBRATION");
  Serial.printf("REFERENCE: %s MOTOR at PWM %d (fixed)\n", FIXED_MOTOR == FIX_LEFT ? "LEFT" : "RIGHT", FIXED_PWM);

  calibrationComplete = false;
  setForwardDirection();

  int bestPWM = constrain(SEARCH_CENTER_PWM, MIN_PWM, MAX_PWM);
  float bestError = 1000000.0f;

  // Coarse search
  Serial.println("\n--- COARSE SEARCH ---");
  delay(500);
  beep(1);

  searchRange(SEARCH_CENTER_PWM - COARSE_OFFSET, SEARCH_CENTER_PWM + COARSE_OFFSET, COARSE_STEP, bestPWM, bestError);

  Serial.printf("\nBest coarse PWM: %d | Error: %.2f %%\n", bestPWM, bestError);

  // Fine search (error reset so fine results are judged on their own)
  Serial.println("\n--- FINE SEARCH ---");
  beep(1);

  bestError = 1000000.0f;
  searchRange(bestPWM - FINE_OFFSET, bestPWM + FINE_OFFSET,  FINE_STEP, bestPWM, bestError);

  applyAdjustablePWM(bestPWM);

  printBanner("BEST PWM FOUND");
  Serial.printf("LEFT PWM  : %d\n", motorAPWM);
  Serial.printf("RIGHT PWM : %d\n", motorBPWM);

  delay(SETTLE_TIME_MS);

  // Final confirmation
  Serial.println("\n--- FINAL CONFIRMATION ---");
  beep(1);

  int confirmations = constrain(FINAL_CONFIRM_SAMPLES, 1, MAX_SAMPLES);
  SpeedMeasurement samples[MAX_SAMPLES];

  for (int i = 0; i < confirmations; i++)
  {
    samples[i] = measureSpeedOnce(CONFIRM_WINDOW_MS);

    Serial.printf("Confirmation %d/%d | L %.2f RPM | R %.2f RPM | Error %.2f %%\n",  i + 1, confirmations, samples[i].rpmA, samples[i].rpmB, speedDifferencePercent(samples[i].rpmA, samples[i].rpmB));
  }

  SpeedMeasurement confirmed = medianSpeed(samples, confirmations);
  float confirmedError = speedDifferencePercent(confirmed.rpmA, confirmed.rpmB);

  printBanner("FINAL CALIBRATION RESULT");
  Serial.printf("LEFT PWM        : %d\n", motorAPWM);
  Serial.printf("RIGHT PWM       : %d\n", motorBPWM);
  Serial.printf("LEFT MEDIAN RPM : %.2f\n", confirmed.rpmA);
  Serial.printf("RIGHT MEDIAN RPM: %.2f\n", confirmed.rpmB);
  Serial.printf("MATCH ERROR     : %.2f %%\n", confirmedError);

  stopAndOptionalBrake();

  if (confirmedError <= TARGET_MATCH_PERCENT)
  {
    calibrationComplete = true;
    saveCalibration(confirmed);

    Serial.println("\n*** CALIBRATION SUCCESSFUL ***");
    beep(3, 2500);

    printBanner("SYSTEM READY");
    Serial.println("Press button for saved-PWM speed test.");
  }
  else
  {
    Serial.println("\n*** CALIBRATION DID NOT REACH TARGET ***");
    Serial.println("Result NOT saved. Check encoders, mechanics and motor supply.");
    Serial.println("Press reset to try again.");
    beep(1);
  }
}

// SPEED TEST
void speedMatchTest()
{
  printBanner("SAVED-PWM SPEED MATCH TEST");

  setForwardDirection();

  motorAPWM = savedMotorAPWM;
  motorBPWM = savedMotorBPWM;
  writeMotorPWM();

  delay(SPEED_TEST_SETTLE_MS);

  int count = constrain(SPEED_TEST_SAMPLES, 1, MAX_SAMPLES);
  SpeedMeasurement samples[MAX_SAMPLES];

  for (int i = 0; i < count; i++)
  {
    samples[i] = measureSpeedOnce(SPEED_TEST_WINDOW_MS);

    Serial.printf("Test %d/%d | LEFT %.2f RPM | RIGHT %.2f RPM | Error %.2f %%\n", i + 1, count, samples[i].rpmA, samples[i].rpmB,  speedDifferencePercent(samples[i].rpmA, samples[i].rpmB));
  }

  SpeedMeasurement median = medianSpeed(samples, count);
  float finalError = speedDifferencePercent(median.rpmA, median.rpmB);

  stopAndOptionalBrake();

  printBanner("SPEED TEST RESULT");
  Serial.printf("LEFT PWM         : %d\n", savedMotorAPWM);
  Serial.printf("RIGHT PWM        : %d\n", savedMotorBPWM);
  Serial.printf("LEFT MEDIAN RPM  : %.2f\n", median.rpmA);
  Serial.printf("RIGHT MEDIAN RPM : %.2f\n", median.rpmB);
  Serial.printf("LEFT MEDIAN PPS  : %.2f\n", median.ppsA);
  Serial.printf("RIGHT MEDIAN PPS : %.2f\n", median.ppsB);
  Serial.printf("SPEED DIFFERENCE : %.2f %%\n", finalError);

  if (finalError <= SPEED_TEST_MATCH_PERCENT)
  {
    Serial.println("\nRESULT: SPEEDS ARE SUCCESSFULLY MATCHED.");
    beep(3, 2500);
  }
  else
  {
    Serial.println("\nRESULT: SPEEDS ARE NOT WITHIN THE TEST LIMIT.");
    beep(1);
  }
}
 
// BUTTON 
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

  if (!pressed)
    return;

  delay(BUTTON_DEBOUNCE);

  if (digitalRead(SWITCH_PIN) != LOW)
    return;

  buttonArmed = false;
  Serial.println("\nBUTTON PRESSED -> SPEED TEST");

  speedMatchTest();

  while (digitalRead(SWITCH_PIN) == LOW)
    delay(5);

  Serial.println("Button released. System ready.");
}

void setup()
{
  beep(2);
  Serial.begin(115200);
  delay(200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(MOTOR_A_ENCODER, INPUT_PULLUP);
  pinMode(MOTOR_B_ENCODER, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT);

  pwmInit();

  digitalWrite(STBY, HIGH);
  setForwardDirection();
  stopMotors();
  resetEncoderCounts();

  attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODER), motorAISR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCODER), motorBISR, RISING);

  beep(2);

  calibrateMotors();
}

void loop()
{
  checkButton();
}