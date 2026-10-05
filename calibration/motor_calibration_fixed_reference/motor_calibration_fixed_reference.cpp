/*
 * MOTOR CALIBRATION (Stage 2 of 3)
 * ESP32 + Arduino framework, PlatformIO
 
 * WHAT IT DOES
  Finds an approximate PWM for the adjustable motor so that both motors spin at the same encoder speed. One motor (FIXED_MOTOR) is held at FIXED_PWM as the reference. The other motor's PWM is adjusted until the speed difference stays within CAL_TOLERANCE_PERCENT for MATCH_CONFIRMATIONS consecutive measurements. The result is printed on he serial monitor and saved to flash. The PWM it a starting point for Stage 3 (refinement), not as a final value.

 * HOW TO USE
  1. Lift the wheels off the ground. Use the same battery level for every run.
  2. Set FIXED_MOTOR and FIXED_PWM in USER SETTINGS.
  3. Upload, then open the Serial Monitor at 115200 baud.
  4. Listen to the buzzer and observe the serial output. The buzzer indicates:
       2 beeps    = system active, motors start about 1 s later
       1 beep     = adjustable motor PWM was just changed
       3 beeps    = calibration finished
       5 beeps   = calibration failed (read the serial message)
  5. Write down the LEFT PWM and RIGHT PWM from the "CALIBRATION SAVED" block. The fixed motor shows
     FIXED_PWM; the other value is your result.
  6. Optional: press the button to run the speed-match test (passes at SPEED_TEST_MATCH_PERCENT or
     less).
  7. Reset the board and repeat steps 4-6 several times. Use the adjustable-motor PWM that appears
     most consistently (or the median) as the input for Stage 3.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <algorithm>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// Choose which motor stays fixed at FIXED_PWM during calibration.
enum FixedMotor
{
  FIX_LEFT,
  FIX_RIGHT
};

const FixedMotor FIXED_MOTOR = FIX_RIGHT;
const int FIXED_PWM = 200;

// true  = calibrate every boot
// false = reuse the saved calibration
const bool RECALIBRATE_ON_BOOT = true;

// Encoder settings
const float MOTOR_A_PULSES_PER_REV = 40.0f;
const float MOTOR_B_PULSES_PER_REV = 40.0f;

// Minimum time between counted pulses (noise filter).
const uint32_t ENCODER_MIN_INTERVAL_US = 200;

// Calibration measurements
const unsigned long CAL_SAMPLE_TIME = 2000;
const int CAL_AVERAGE_SAMPLES = 3;
const unsigned long CAL_SETTLE_TIME = 1000;
const float CAL_TOLERANCE_PERCENT = 3.0f;
const int MATCH_CONFIRMATIONS = 4;
const int CAL_MIN_PWM = 0;
const int CAL_MAX_PWM = 255;
const int CAL_MIN_STEP = 1;
const int CAL_MAX_STEP = 25;
const float CAL_GAIN = 0.255f;
const int CAL_MAX_CORRECTION = 150;
const int CAL_MAX_ITERATIONS = 60;
const int CAL_MAX_ZERO_SPEED_RETRIES = 3;
const int CAL_MAX_SATURATED = 3;

const uint32_t PWM_FREQ_HZ = 1000;
const uint8_t PWM_RESOLUTION_BITS = 8;

const int channelA = 0;
const int channelB = 1;
const int channelBuzzer = 2;

// Speed test settings 
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

// Button debounce and brake settings
bool activateBrakes = false;
const unsigned long BUTTON_DEBOUNCE = 30;
const unsigned long BRAKE_TIME = 60;

// Pins
constexpr uint8_t PWMA = 25;
constexpr uint8_t AIN1 = 26;
constexpr uint8_t AIN2 = 27;
constexpr uint8_t PWMB = 14;
constexpr uint8_t BIN1 = 4;
constexpr uint8_t BIN2 = 13;
constexpr uint8_t STBY = 33;
constexpr uint8_t MOTOR_A_ENCODER = 18;
constexpr uint8_t MOTOR_B_ENCODER = 19;
constexpr uint8_t BUZZER_PIN = 23;
constexpr uint8_t SWITCH_PIN = 35;

// Globals
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t motorACount = 0;
volatile uint32_t motorBCount = 0;
volatile uint32_t lastAEncoderMicros = 0;
volatile uint32_t lastBEncoderMicros = 0;
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

// Encoder Interupts

void IRAM_ATTR motorAISR()
{
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&encoderMux);
  if (now - lastAEncoderMicros >= ENCODER_MIN_INTERVAL_US)
  {
    motorACount = motorACount + 1;
    lastAEncoderMicros = now;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR motorBISR()
{
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&encoderMux);
  if (now - lastBEncoderMicros >= ENCODER_MIN_INTERVAL_US)
  {
    motorBCount = motorBCount + 1;
    lastBEncoderMicros = now;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// Ledc (PWM) Wrappers

void pwmAttach(uint8_t pin, int channel)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(channel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(uint8_t pin, int channel, uint32_t duty)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void buzzerAttach()
{
  pwmAttach(BUZZER_PIN, channelBuzzer);
}

void buzzerTone(uint32_t freq)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, freq);
#else
  ledcWriteTone(channelBuzzer, freq);
#endif
}

// Buzzer Functions
void beep(uint32_t freq, unsigned long ms)
{
  buzzerTone(freq);
  delay(ms);
  buzzerTone(0);
}

void beepOnce()
{
  beep(3500, 100);
}

void beepTwice(){
  for (int i = 0; i < 2; i++)
  {
    beep(2500, 100);

    if (i == 0)
      delay(100);
  }
}

void beepTriple()
{
  for (int i = 0; i < 3; i++)
  {
    beep(2500, 100);

    if (i < 2)
      delay(100);
  }
}

void beepFail()
{
  for (int i = 0; i < 5; i++)
  {
    beep(2500, 100);

    if (i < 4)
      delay(700);
  }
}


// Motor Control
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
  pwmWrite(PWMA, channelA, motorAPWM);
  pwmWrite(PWMB, channelB, motorBPWM);
}

void stopMotors()
{
  pwmWrite(PWMA, channelA, 0);
  pwmWrite(PWMB, channelB, 0);
}

// Brakes both motors at the same time (single BRAKE_TIME wait).
void brakeMotors()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);

  pwmWrite(PWMA, channelA, 255);
  pwmWrite(PWMB, channelB, 255);
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

// Encoder Count Management
void resetAllEncoderCounts()
{
  const uint32_t now = micros();
  portENTER_CRITICAL(&encoderMux);
  motorACount = 0;
  motorBCount = 0;
  lastAEncoderMicros = now;
  lastBEncoderMicros = now;
  portEXIT_CRITICAL(&encoderMux);
}

void readAndClearBothCounts(uint32_t &countA, uint32_t &countB)
{
  portENTER_CRITICAL(&encoderMux);
  countA = motorACount;
  countB = motorBCount;
  motorACount = 0;
  motorBCount = 0;
  portEXIT_CRITICAL(&encoderMux);
}

// Speed Measurement
SpeedMeasurement measureSpeedOnce(unsigned long durationMs)
{
  SpeedMeasurement result;

  const uint32_t durationUs = (uint32_t)durationMs * 1000UL;

  resetAllEncoderCounts();
  const uint32_t startUs = micros();

  while ((uint32_t)(micros() - startUs) < durationUs)
  {
    delay(1);
  }

  uint32_t countA = 0;
  uint32_t countB = 0;
  readAndClearBothCounts(countA, countB);

  // Use the real elapsed time rather than the nominal duration.
  const float seconds = (float)(uint32_t)(micros() - startUs) / 1.0e6f;

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
  const float average = (speedA + speedB) / 2.0f;

  if (average <= 0.0f)
    return 100.0f;

  return (fabsf(speedA - speedB) / average) * 100.0f;
}

// Calibration Storage
void saveCalibration(const SpeedMeasurement &speed)
{
  savedMotorAPWM = motorAPWM;
  savedMotorBPWM = motorBPWM;
  savedMotorAPPS = speed.ppsA;
  savedMotorBPPS = speed.ppsB;
  savedMotorARPM = speed.rpmA;
  savedMotorBRPM = speed.rpmB;

  if (preferences.begin("motorCal", false))
  {
    preferences.putInt("motorAPWM", savedMotorAPWM);
    preferences.putInt("motorBPWM", savedMotorBPWM);
    preferences.putFloat("motorAPPS", savedMotorAPPS);
    preferences.putFloat("motorBPPS", savedMotorBPPS);
    preferences.putFloat("motorARPM", savedMotorARPM);
    preferences.putFloat("motorBRPM", savedMotorBRPM);
    preferences.putUChar("fixedMotor", (uint8_t)FIXED_MOTOR);
    preferences.putInt("fixedPWM", FIXED_PWM);
    preferences.putBool("valid", true);
    preferences.end();
  }
  else
  {
    Serial.println("WARNING: could not open NVS - calibration not stored.");
  }

  Serial.println();
  Serial.println(" ");
  Serial.println("CALIBRATION SAVED");
  Serial.println(" ");
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

bool loadCalibration()
{
  if (!preferences.begin("motorCal", true))
    return false;

  bool ok = preferences.getBool("valid", false);

  if (ok)
  {
    ok = (preferences.getUChar("fixedMotor", 255) == (uint8_t)FIXED_MOTOR) &&
         (preferences.getInt("fixedPWM", -1) == FIXED_PWM);
  }

  if (ok)
  {
    savedMotorAPWM = preferences.getInt("motorAPWM", FIXED_PWM);
    savedMotorBPWM = preferences.getInt("motorBPWM", FIXED_PWM);
    savedMotorAPPS = preferences.getFloat("motorAPPS", 0.0f);
    savedMotorBPPS = preferences.getFloat("motorBPPS", 0.0f);
    savedMotorARPM = preferences.getFloat("motorARPM", 0.0f);
    savedMotorBRPM = preferences.getFloat("motorBRPM", 0.0f);
  }

  preferences.end();

  if (ok)
  {
    Serial.println();
    Serial.println("Loaded saved calibration:");
    Serial.print("LEFT PWM  : ");
    Serial.println(savedMotorAPWM);
    Serial.print("RIGHT PWM : ");
    Serial.println(savedMotorBPWM);
  }

  return ok;
}

// Calibration Process
bool failCalibration(const char *reason)
{
  stopAndOptionalBrake();
  calibrationComplete = false;
  Serial.println();
  Serial.println("*** CALIBRATION FAILED ***");
  Serial.println(reason);
  beepFail();
  return false;
}

bool calibrateMotors()
{
  Serial.println();
  Serial.println(" ");
  Serial.println("MOTOR CALIBRATION");
  Serial.println(" ");

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

  const int pwmLow  = std::max<int>(CAL_MIN_PWM, FIXED_PWM - CAL_MAX_CORRECTION);
  const int pwmHigh = std::min<int>(CAL_MAX_PWM, FIXED_PWM + CAL_MAX_CORRECTION);

  int adjustablePWM = FIXED_PWM;
  int consecutiveMatches = 0;
  int zeroSpeedCount = 0;
  int saturatedCount = 0;

  for (int iteration = 1; iteration <= CAL_MAX_ITERATIONS; iteration++)
  {
    SpeedMeasurement speed = measureAverageSpeed();

    const float matchError = speedDifferencePercent(speed.ppsA, speed.ppsB);

    Serial.println();
    Serial.print("--- Calibration measurement ");
    Serial.print(iteration);
    Serial.print("/");
    Serial.print(CAL_MAX_ITERATIONS);
    Serial.println(" ---");

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

      if (++zeroSpeedCount >= CAL_MAX_ZERO_SPEED_RETRIES)
        return failCalibration("No encoder pulses from one or both motors.");

      delay(300);
      continue;
    }

    zeroSpeedCount = 0;

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
        return true;
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
    const float relativeError = (adjustableSpeed - fixedSpeed) / fixedSpeed;

    int step = (int)roundf(fabsf(relativeError) * FIXED_PWM * CAL_GAIN);
    step = constrain(step, CAL_MIN_STEP, CAL_MAX_STEP);

    int desiredPWM = adjustablePWM;

    if (relativeError > 0.0f)
    {
      desiredPWM -= step;
      Serial.print("Adjustable motor is FASTER -> PWM -");
      Serial.println(step);
    }
    else
    {
      desiredPWM += step;
      Serial.print("Adjustable motor is SLOWER -> PWM +");
      Serial.println(step);
    }

    adjustablePWM = constrain(desiredPWM, pwmLow, pwmHigh);

    // Pinned at the PWM limit and still not matching?
    if (adjustablePWM != desiredPWM)
    {
      Serial.println("Adjustable PWM is at its limit.");

      if (++saturatedCount >= CAL_MAX_SATURATED)
        return failCalibration("Adjustable motor cannot reach the reference speed. " "Lower FIXED_PWM (more headroom), inspect connections or swap FIXED_MOTOR.");
    }
    else
    {
      saturatedCount = 0;
    }

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

  return failCalibration("Did not converge within CAL_MAX_ITERATIONS.");
}

// SPEED TEST BUTTON ACTION
void speedMatchTest()
{
  Serial.println();
  Serial.println(" ");
  Serial.println("MOTOR SPEED MATCH TEST");
  Serial.println(" ");
  Serial.println("Both motors will run at their saved PWM.");
  Serial.println();

  stopMotors();
  setForwardDirection();
  delay(SPEED_TEST_SETTLE_TIME);

  motorAPWM = savedMotorAPWM;
  motorBPWM = savedMotorBPWM;
  writeMotorPWM();

  Serial.println("Letting motors settle (1 s)...");
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

  const float avgPPSA = totalPPSA / SPEED_TEST_SAMPLES;
  const float avgPPSB = totalPPSB / SPEED_TEST_SAMPLES;
  const float avgRPMA = totalRPMA / SPEED_TEST_SAMPLES;
  const float avgRPMB = totalRPMB / SPEED_TEST_SAMPLES;
  const float finalError = speedDifferencePercent(avgPPSA, avgPPSB);

  Serial.println(" ");
  Serial.println("FINAL SPEED TEST RESULT");
  Serial.println(" ");
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

// BUTTON HANDLING
void checkButton()
{
  if (!calibrationComplete)
    return;

  const bool pressed = (digitalRead(SWITCH_PIN) == LOW);

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

void setup()
{
  Serial.begin(115200);
  delay(1500);  // lets the serial monitor reconnect after upload/reset
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(MOTOR_A_ENCODER, INPUT_PULLUP);
  pinMode(MOTOR_B_ENCODER, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT);

  pwmAttach(PWMA, channelA);
  pwmAttach(PWMB, channelB);
  buzzerAttach();
  digitalWrite(STBY, HIGH);
  setForwardDirection();
  stopMotors();
  resetAllEncoderCounts();

  attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODER), motorAISR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCODER), motorBISR, RISING);

  beepTwice();
  delay(300);

  bool ready = false;

  if (!RECALIBRATE_ON_BOOT && loadCalibration())
  {
    calibrationComplete = true;
    buttonArmed = false;
    ready = true;
  }
  else
  {
    ready = calibrateMotors();
  }

  if (!ready)
  {
    Serial.println();
    Serial.println("System halted - fix the issue above and reset the board.");
    return;
  }

  Serial.println();
  Serial.println(" ");
  Serial.println("SYSTEM READY");
  Serial.println(" ");
  Serial.println("Press button for SPEED MATCH TEST.");
  Serial.print("Reference motor: ");
  Serial.println(FIXED_MOTOR == FIX_LEFT ? "LEFT" : "RIGHT");
  Serial.print("Reference PWM: ");
  Serial.println(FIXED_PWM);
  Serial.print("Brake system: ");
  Serial.println(activateBrakes ? "ENABLED" : "DISABLED");
}

void loop()
{
  checkButton();
  delay(5);
}