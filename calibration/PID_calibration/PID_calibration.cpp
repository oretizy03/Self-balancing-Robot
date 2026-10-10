/*
 * PID Balancing (ESP32 self-balancing robot)

 * WHAT IT DOES:
    Balances the robot using the saved MPU6050 calibration (MpuCal.h) and the motor speed match (RIGHT_REF_PWM, lLEFT_MATCH_PWM). It contains no calibration routines. PID gains can be tuned live over serial.

 * HOW TO USE:
    1. Run the MPU calibration tool first (it saves to flash).
    2. Make sure to have MpuCal.h in include
    3. Upload. Keep the robot still until the "ready" beep, then hold it upright. It engages after 1 s of steady hold.
    Serial (115200, ENTER to send):
       pVALUE, iVALUE, dVALUE, tVALUE - set Kp / Ki / Kd e.g. p22.5 . Balance trim in deg (+ = balances leaning forward)
       s - save PID + trim to flash
       v - telemetry on/off
       ? - print current values

 * BEEPS:
    2 short - startup (before anything is loaded)
    1 medium - saved calibration loaded
    1 short - gyro zeroed, ready: hold the robot upright
    2 long - gyro zero rejected (robot moved), saved bias used
    3 quick - balancing engaged
    1 long - fell over, motors off
    2 quick - PID settings saved
    3 long, repeating - saved calibration missing or invalid (halted)
    5 quick, repeating - MPU6050 not found (halted)

 * NOTE:
    Positive angle = leaning forward (same as the calibration tool's live mode). Loop rate and filter match the calibration tool (250 Hz, 0.98). Set GYRO_ZERO_AT_BOOT to 0 to skip the boot-time gyro bias refresh.
*/

#include <Arduino.h>
#include "MpuCal.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define CORE3 1
#else
  #define CORE3 0
#endif

// Settings
// Motor A = LEFT, Motor B = RIGHT
#define PWMA 25
#define AIN1 26
#define AIN2 27
#define PWMB 14
#define BIN1 4
#define BIN2 13
#define STBY 33
#define BUZZER_PIN 23
#define BUZZER_PASSIVE 1        // 0 for an active buzzer
#define BUZZER_FREQ 2500
#define PWM_FREQ 1000
#define LEFT_DIR 1        // -1 if that wheel drives the wrong way
#define RIGHT_DIR 1
#define RIGHT_REF_PWM 200
#define LEFT_MATCH_PWM 140
#define SAMPLE_HZ 250
#define GYRO_ZERO_AT_BOOT  1
#define GYRO_ZERO_MAX_DELTA_DPS 2.0f   // reject refresh if it moved more than this
#define UPRIGHT_WINDOW 5.0f     // deg from balance point to start engaging
#define UPRIGHT_HOLD_MS 1000
#define FALL_ANGLE 35.0f    // deg from balance point to cut motors
#define I_MAX 80.0f

static const float ALPHA = 0.98f;
static const float LEFT_SCALE = (float)LEFT_MATCH_PWM / RIGHT_REF_PWM;
static const uint32_t LOOP_US = 1000000UL / SAMPLE_HZ;

// PID starting values, tune over serial
static float kp = 20.0f, ki = 0.0f, kd = 0.8f, trim = 0.0f;

#define LEFT_CH 0
#define RIGHT_CH 1
#define BUZ_CH 2

Adafruit_MPU6050 mpu;
MpuCalibration cal;

static float angle = 0.0f, iTerm = 0.0f, lastOut = 0.0f;
static bool engaged = false, holding = false, telemetry = false;
static uint32_t holdStart = 0, lastUs = 0, lastTel = 0;

// PWM helpers
static void pwmAttach(int pin, int ch, int freq)
{
#if CORE3
  ledcAttach(pin, freq, 8);
#else
  ledcSetup(ch, freq, 8);
  ledcAttachPin(pin, ch);
#endif
}

static void pwmWrite(int pin, int ch, int duty)
{
#if CORE3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// Buzzer (non-blocking)
static int bzLeft = 0, bzOn = 0, bzOff = 0;
static bool bzState = false;
static uint32_t bzT = 0;

static void buzz(bool on)
{
#if BUZZER_PASSIVE
  pwmWrite(BUZZER_PIN, BUZ_CH, on ? 128 : 0);
#else
  digitalWrite(BUZZER_PIN, on);
#endif
}

static void buzzerInit()
{
#if BUZZER_PASSIVE
  pwmAttach(BUZZER_PIN, BUZ_CH, BUZZER_FREQ);
#else
  pinMode(BUZZER_PIN, OUTPUT);
#endif
  buzz(false);
}

static void beep(int n, int onMs = 80, int offMs = 80)
{
  bzLeft = n;
  bzOn = onMs;
  bzOff = offMs;
  bzState = true;
  bzT = millis();
  buzz(true);
}

static void beepUpdate()
{
  if (bzLeft <= 0) return;

  uint32_t now = millis();
  if (bzState && now - bzT >= (uint32_t)bzOn)
  {
    buzz(false);
    bzState = false;
    bzT = now;
    bzLeft--;
  }
  else if (!bzState && bzLeft > 0 && now - bzT >= (uint32_t)bzOff)
  {
    buzz(true);
    bzState = true;
    bzT = now;
  }
}

static void beepWait()
{
  while (bzLeft > 0)
  {
    beepUpdate();
    delay(1);
  }
}

// Motors
static void setMotor(int pwmPin, int ch, int in1, int in2, int v)
{
  digitalWrite(in1, v > 0);
  digitalWrite(in2, v < 0);
  pwmWrite(pwmPin, ch, constrain(abs(v), 0, 255));
}

static void motorsStop()
{
  setMotor(PWMA, LEFT_CH,  AIN1, AIN2, 0);
  setMotor(PWMB, RIGHT_CH, BIN1, BIN2, 0);
  digitalWrite(STBY, LOW);
}

static void motorsInit()
{
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  const int dirPins[] = {AIN1, AIN2, BIN1, BIN2};
  for (int p : dirPins)
  {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  pwmAttach(PWMA, LEFT_CH, PWM_FREQ);
  pwmAttach(PWMB, RIGHT_CH, PWM_FREQ);
  motorsStop();
}

static void drive(float u)
{
  int r = (int)lroundf(u) * RIGHT_DIR;
  int l = (int)lroundf(u * LEFT_SCALE) * LEFT_DIR;

  digitalWrite(STBY, HIGH);
  setMotor(PWMB, RIGHT_CH, BIN1, BIN2, r);
  setMotor(PWMA, LEFT_CH,  AIN1, AIN2, l);
}

[[noreturn]] static void fault(int n, int onMs, int offMs)
{
  motorsStop();
  for (;;)
  {
    beep(n, onMs, offMs);
    beepWait();
    delay(1500);
  }
}

// IMU
// accAngle: accelerometer angle relative to the balance point (deg)
// rate: pitch rate, positive = leaning forward (deg/s)
static void readImu(float &accAngle, float &rate)
{
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  mpuCorrectAccel(cal, ax, ay, az);

  accAngle = wrap180(accelPitchDeg(ax, ay, az) - cal.balanceAngle);
  rate = (g.gyro.y - cal.gyroBias[1]) * RAD_TO_DEG;
}

// PID settings in flash
static void loadPid()
{
  Preferences p;
  if (!p.begin("pid", true)) return;
  kp   = p.getFloat("kp", kp);
  ki   = p.getFloat("ki", ki);
  kd   = p.getFloat("kd", kd);
  trim = p.getFloat("trim", trim);
  p.end();
}

static void savePid()
{
  Preferences p;
  p.begin("pid", false);
  p.putFloat("kp", kp);
  p.putFloat("ki", ki);
  p.putFloat("kd", kd);
  p.putFloat("trim", trim);
  p.end();
}

// Serial tuning
static void printSettings()
{
  Serial.printf("Kp=%.3f Ki=%.3f Kd=%.3f trim=%.2f | balance=%.3f deg gyroBiasY=%.4f deg/s\n", kp, ki, kd, trim, cal.balanceAngle, cal.gyroBias[1] * RAD_TO_DEG);
}

static void handleCmd(char *s)
{
  float v = atof(s + 1);
  switch (tolower(s[0]))
  {
    case 'p': kp = v; break;
    case 'i': ki = v; iTerm = 0.0f; break;
    case 'd': kd = v; break;
    case 't': trim = v; break;
    case 's': savePid(); beep(2, 60, 60); break;
    case 'v': telemetry = !telemetry; break;
    case '?': break;
    default: return;
  }
  printSettings();
}

static void readSerial()
{
  static char buf[24];
  static uint8_t n = 0;

  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (n)
      {
        buf[n] = 0;
        handleCmd(buf);
        n = 0;
      }
    }
    else if (n < sizeof(buf) - 1)
    {
      buf[n++] = c;
    }
  }
}

void setup()
{
  Serial.begin(115200);

  motorsInit();
  buzzerInit();

  beep(2, 100, 100);
  beepWait();
  delay(700);

  if (!mpuCalLoad(cal)) fault(3, 400, 250);
  loadPid();
  beep(1, 200);
  beepWait();

  if (!mpuBegin(mpu)) fault(5, 80, 80);

  bool zeroed = true;
#if GYRO_ZERO_AT_BOOT
  MpuCalibration fresh = cal;
  mpuQuickGyroZero(mpu, fresh);
  float delta = fabsf(fresh.gyroBias[1] - cal.gyroBias[1]) * RAD_TO_DEG;
  if (delta <= GYRO_ZERO_MAX_DELTA_DPS) cal = fresh;
  else zeroed = false;
#endif
  if (zeroed) beep(1, 80);
  else        beep(2, 400, 200);
  beepWait();

  float rate;
  readImu(angle, rate);

  printSettings();
  lastUs = micros();
}

void loop()
{
  beepUpdate();
  readSerial();

  uint32_t now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;

  float accAngle, rate;
  readImu(accAngle, rate);

  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * accAngle;
  if (!isfinite(angle)) fault(5, 80, 80);

  float err = angle - trim;

  if (!engaged)
  {
    motorsStop();
    lastOut = 0.0f;

    if (fabsf(err) < UPRIGHT_WINDOW)
    {
      if (!holding)
      {
        holding = true;
        holdStart = millis();
      }
      else if (millis() - holdStart >= UPRIGHT_HOLD_MS)
      {
        iTerm = 0.0f;
        engaged = true;
        holding = false;
        beep(3, 80, 80);
      }
    }
    else
    {
      holding = false;
    }
  }
  else if (fabsf(err) > FALL_ANGLE)
  {
    motorsStop();
    engaged = false;
    holding = false;
    iTerm = 0.0f;
    lastOut = 0.0f;
    beep(1, 700);
  }
  else
  {
    iTerm = constrain(iTerm + ki * err * dt, -I_MAX, I_MAX);
    float u = kp * err + iTerm + kd * rate;
    u = constrain(u, -255.0f, 255.0f);
    lastOut = u;
    drive(u);
  }

  if (telemetry && millis() - lastTel >= 100)
  {
    lastTel = millis();
    Serial.printf("ANGLE=%8.3f  ACCEL=%8.3f  RATE=%9.3f  OUT=%5.0f  %s\n", angle, accAngle, rate, lastOut, engaged ? "RUN" : "WAIT");
  }
}