/*
  ESP32 + MPU6050
  Self-Balancing Robot Calibration & PID Data Generator

  ESP32:
    SDA = GPIO 21
    SCL = GPIO 22

  Calibration:
    1. 6-face accelerometer calibration
       +X UP
       -X UP
       +Y UP
       -Y UP
       +Z UP
       -Z UP

    2. Gyroscope bias calibration while completely stationary

    3. Balance-position zero calibration

    4. Stationary noise + timing analysis

    5. Live PID-ready angle/rate output

  NOTE:
    Do the 6-face calibration carefully.
    Keep the MPU6050 completely still during each measurement.

  Serial Monitor:
    115200 baud
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <math.h>

// ============================================================
// ESP32 I2C PINS
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22

// ============================================================
// MPU SETTINGS
// ============================================================

#define MPU_ADDRESS_1 0x68
#define MPU_ADDRESS_2 0x69

// Accelerometer: +/-2G gives highest resolution
#define ACCEL_RANGE MPU6050_RANGE_2_G

// Gyroscope: +/-500 deg/s is suitable for balancing robots
#define GYRO_RANGE MPU6050_RANGE_500_DEG

// Digital low-pass filter
// 44 Hz gives a reasonable balance between noise reduction
// and response speed for a balancing robot.
#define DLPF_BANDWIDTH MPU6050_BAND_44_HZ

// ============================================================
// CALIBRATION SETTINGS
// ============================================================

#define ACCEL_CAL_SAMPLES 1200
#define GYRO_CAL_SAMPLES 2000
#define ZERO_CAL_SAMPLES 1000
#define NOISE_TEST_SAMPLES 2000

#define CAL_SAMPLE_HZ 250
#define TEST_SAMPLE_HZ 250

#define GRAVITY 9.80665f

// ============================================================
// SELF-BALANCING ROBOT AXIS SELECTION
// ============================================================
//
// Most self-balancing robots use PITCH.
//
// If your MPU6050 is mounted differently, change:
// AXIS_PITCH -> AXIS_ROLL
//
// PITCH:
//   Angle = atan2(-X, sqrt(Y² + Z²))
//   Gyro  = Y gyro
//
// ROLL:
//   Angle = atan2(Y, Z)
//   Gyro  = X gyro
// ============================================================

enum BalanceAxis
{
  AXIS_PITCH,
  AXIS_ROLL
};

BalanceAxis BALANCE_AXIS = AXIS_PITCH;

// ============================================================
// COMPLEMENTARY FILTER
// ============================================================

float complementaryAlpha = 0.98f;

// ============================================================
// EEPROM / FLASH STORAGE
// ============================================================

Preferences preferences;

#define CAL_MAGIC 0x6050ABCD

struct CalibrationData
{
  uint32_t magic;

  // Accelerometer bias
  float accelBiasX;
  float accelBiasY;
  float accelBiasZ;

  // Accelerometer scale
  float accelScaleX;
  float accelScaleY;
  float accelScaleZ;

  // Gyroscope bias
  float gyroBiasX;
  float gyroBiasY;
  float gyroBiasZ;

  // Mechanical balance zero
  float balanceZeroPitch;
  float balanceZeroRoll;
};

CalibrationData cal;

// ============================================================
// MPU OBJECT
// ============================================================

Adafruit_MPU6050 mpu;

uint8_t mpuAddress = 0;

// ============================================================
// SENSOR DATA
// ============================================================

struct SensorData
{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float temperature;
};

// ============================================================
// STATISTICS
// ============================================================

struct Statistics
{
  double mean;
  double M2;

  float minValue;
  float maxValue;

  uint32_t count;
};

void statsReset(Statistics &s)
{
  s.mean = 0.0;
  s.M2 = 0.0;

  s.minValue = 999999.0f;
  s.maxValue = -999999.0f;

  s.count = 0;
}

void statsAdd(Statistics &s, float value)
{
  s.count++;

  double delta = value - s.mean;

  s.mean += delta / s.count;

  double delta2 = value - s.mean;

  s.M2 += delta * delta2;

  if (value < s.minValue)
    s.minValue = value;

  if (value > s.maxValue)
    s.maxValue = value;
}

float statsMean(const Statistics &s)
{
  return (float)s.mean;
}

float statsStdDev(const Statistics &s)
{
  if (s.count < 2)
    return 0.0f;

  return sqrt((float)(s.M2 / (s.count - 1)));
}

float statsPeakToPeak(const Statistics &s)
{
  return s.maxValue - s.minValue;
}

// ============================================================
// FIXED RATE SAMPLING
// ============================================================

void waitForNextSample(uint32_t &nextSample, uint32_t intervalUs)
{
  while ((int32_t)(micros() - nextSample) < 0)
  {
    yield();
  }

  nextSample += intervalUs;

  // Prevent runaway if processing took too long
  uint32_t now = micros();

  if ((int32_t)(now - nextSample) > (int32_t)intervalUs)
  {
    nextSample = now + intervalUs;
  }
}

// ============================================================
// SERIAL INPUT
// ============================================================

void clearSerialBuffer()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

void waitForUser()
{
  clearSerialBuffer();

  while (!Serial.available())
  {
    delay(10);
  }

  clearSerialBuffer();

  delay(200);
}

// ============================================================
// PRINT HEADER
// ============================================================

void printHeader()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("       ESP32 MPU6050 SELF-BALANCING ROBOT CALIBRATION");
  Serial.println("============================================================");
  Serial.println();

  Serial.println("I2C:");
  Serial.println("  SDA = GPIO 21");
  Serial.println("  SCL = GPIO 22");
  Serial.print("  MPU6050 address = 0x");
  Serial.println(mpuAddress, HEX);

  Serial.println();

  Serial.println("Sensor configuration:");
  Serial.println("  Accelerometer = +/-2G");
  Serial.println("  Gyroscope     = +/-500 deg/s");
  Serial.println("  DLPF          = 44 Hz");
  Serial.println("  Calibration   = 250 Hz");
  Serial.println();
}

// ============================================================
// SENSOR READ
// ============================================================

void readSensor(SensorData &s)
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);

  s.ax = accel.acceleration.x;
  s.ay = accel.acceleration.y;
  s.az = accel.acceleration.z;

  // Adafruit returns gyro in rad/s
  s.gx = gyro.gyro.x;
  s.gy = gyro.gyro.y;
  s.gz = gyro.gyro.z;

  s.temperature = temp.temperature;
}

// ============================================================
// CALIBRATED SENSOR VALUES
// ============================================================

float correctAccelX(float x)
{
  return (x - cal.accelBiasX) * cal.accelScaleX;
}

float correctAccelY(float y)
{
  return (y - cal.accelBiasY) * cal.accelScaleY;
}

float correctAccelZ(float z)
{
  return (z - cal.accelBiasZ) * cal.accelScaleZ;
}

float gyroRadToDeg(float rad)
{
  return rad * 180.0f / PI;
}

float correctGyroX(float gx)
{
  return gyroRadToDeg(gx - cal.gyroBiasX);
}

float correctGyroY(float gy)
{
  return gyroRadToDeg(gy - cal.gyroBiasY);
}

float correctGyroZ(float gz)
{
  return gyroRadToDeg(gz - cal.gyroBiasZ);
}

// ============================================================
// ACCELEROMETER ANGLES
// ============================================================

float calculatePitch(float ax, float ay, float az)
{
  return atan2f(
           -ax,
           sqrtf(ay * ay + az * az)
         ) * 180.0f / PI;
}

float calculateRoll(float ax, float ay, float az)
{
  return atan2f(
           ay,
           az
         ) * 180.0f / PI;
}

// ============================================================
// SELECT BALANCE ANGLE
// ============================================================

float getBalanceAngle(float pitch, float roll)
{
  if (BALANCE_AXIS == AXIS_PITCH)
  {
    return pitch - cal.balanceZeroPitch;
  }
  else
  {
    return roll - cal.balanceZeroRoll;
  }
}

float getBalanceRate(float gx, float gy)
{
  if (BALANCE_AXIS == AXIS_PITCH)
  {
    return gy;
  }
  else
  {
    return gx;
  }
}

// ============================================================
// I2C DETECTION
// ============================================================

bool detectMPU()
{
  Serial.println("Scanning for MPU6050...");

  Wire.beginTransmission(MPU_ADDRESS_1);

  if (Wire.endTransmission() == 0)
  {
    mpuAddress = MPU_ADDRESS_1;

    Serial.println("MPU6050 found at 0x68");
    return true;
  }

  Wire.beginTransmission(MPU_ADDRESS_2);

  if (Wire.endTransmission() == 0)
  {
    mpuAddress = MPU_ADDRESS_2;

    Serial.println("MPU6050 found at 0x69");
    return true;
  }

  Serial.println("ERROR: MPU6050 not found!");
  return false;
}

// ============================================================
// MPU INITIALIZATION
// ============================================================

bool initializeMPU()
{
  if (!mpu.begin(mpuAddress, &Wire))
  {
    Serial.println("ERROR: MPU6050 initialization failed!");
    return false;
  }

  mpu.setAccelerometerRange(ACCEL_RANGE);
  mpu.setGyroRange(GYRO_RANGE);
  mpu.setFilterBandwidth(DLPF_BANDWIDTH);

  delay(500);

  return true;
}

// ============================================================
// SHOW PROGRESS
// ============================================================

void printProgress(uint32_t current, uint32_t total, int &lastPercent)
{
  int percent = (int)(((current + 1) * 100UL) / total);

  if (percent != lastPercent)
  {
    Serial.print("\rProgress: ");
    Serial.print(percent);
    Serial.print("%   ");

    lastPercent = percent;
  }
}

// ============================================================
// SIX-FACE ACCEL CALIBRATION
// ============================================================

bool collectAccelFace(
  const char *name,
  float &meanX,
  float &meanY,
  float &meanZ
)
{
  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.print("POSITION: ");
  Serial.println(name);
  Serial.println("------------------------------------------------------------");

  Serial.println();
  Serial.println("Place the MPU6050 in the requested orientation.");
  Serial.println("Keep it COMPLETELY STILL.");
  Serial.println();
  Serial.println("Press ENTER when ready...");

  waitForUser();

  Serial.println();
  Serial.println("Settling sensor...");
  delay(1500);

  Statistics xStats;
  Statistics yStats;
  Statistics zStats;

  statsReset(xStats);
  statsReset(yStats);
  statsReset(zStats);

  uint32_t intervalUs = 1000000UL / CAL_SAMPLE_HZ;

  uint32_t nextSample = micros();

  int lastPercent = -1;

  for (uint32_t i = 0; i < ACCEL_CAL_SAMPLES; i++)
  {
    waitForNextSample(nextSample, intervalUs);

    SensorData s;
    readSensor(s);

    statsAdd(xStats, s.ax);
    statsAdd(yStats, s.ay);
    statsAdd(zStats, s.az);

    printProgress(i, ACCEL_CAL_SAMPLES, lastPercent);
  }

  Serial.println();

  meanX = statsMean(xStats);
  meanY = statsMean(yStats);
  meanZ = statsMean(zStats);

  Serial.println();
  Serial.print("Average X = ");
  Serial.print(meanX, 6);
  Serial.println(" m/s^2");

  Serial.print("Average Y = ");
  Serial.print(meanY, 6);
  Serial.println(" m/s^2");

  Serial.print("Average Z = ");
  Serial.print(meanZ, 6);
  Serial.println(" m/s^2");

  Serial.println();

  Serial.print("Noise X = ");
  Serial.print(statsStdDev(xStats), 6);
  Serial.println(" m/s^2");

  Serial.print("Noise Y = ");
  Serial.print(statsStdDev(yStats), 6);
  Serial.println(" m/s^2");

  Serial.print("Noise Z = ");
  Serial.print(statsStdDev(zStats), 6);
  Serial.println(" m/s^2");

  // Basic movement warning
  float totalNoise =
    statsStdDev(xStats) +
    statsStdDev(yStats) +
    statsStdDev(zStats);

  if (totalNoise > 0.20f)
  {
    Serial.println();
    Serial.println("WARNING: Significant movement detected.");
    Serial.println("The result may be less accurate.");
  }

  return true;
}

// ============================================================
// RUN 6-FACE ACCEL CALIBRATION
// ============================================================

void calibrateAccelerometer()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("              6-FACE ACCELEROMETER CALIBRATION");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("You will place the sensor in six positions.");
  Serial.println("Use the axis markings printed on your MPU6050.");
  Serial.println();

  float posX[3];
  float negX[3];

  float posY[3];
  float negY[3];

  float posZ[3];
  float negZ[3];

  // ----------------------------------------------------------
  // +X
  // ----------------------------------------------------------

  collectAccelFace(
    "+X UP  (X axis pointing toward the ceiling)",
    posX[0],
    posX[1],
    posX[2]
  );

  // ----------------------------------------------------------
  // -X
  // ----------------------------------------------------------

  collectAccelFace(
    "-X UP  (X axis pointing toward the floor)",
    negX[0],
    negX[1],
    negX[2]
  );

  // ----------------------------------------------------------
  // +Y
  // ----------------------------------------------------------

  collectAccelFace(
    "+Y UP  (Y axis pointing toward the ceiling)",
    posY[0],
    posY[1],
    posY[2]
  );

  // ----------------------------------------------------------
  // -Y
  // ----------------------------------------------------------

  collectAccelFace(
    "-Y UP  (Y axis pointing toward the floor)",
    negY[0],
    negY[1],
    negY[2]
  );

  // ----------------------------------------------------------
  // +Z
  // ----------------------------------------------------------

  collectAccelFace(
    "+Z UP  (Z axis pointing toward the ceiling)",
    posZ[0],
    posZ[1],
    posZ[2]
  );

  // ----------------------------------------------------------
  // -Z
  // ----------------------------------------------------------

  collectAccelFace(
    "-Z UP  (Z axis pointing toward the floor)",
    negZ[0],
    negZ[1],
    negZ[2]
  );

  // ----------------------------------------------------------
  // CALCULATE BIAS
  // ----------------------------------------------------------

  cal.accelBiasX = (posX[0] + negX[0]) / 2.0f;
  cal.accelBiasY = (posY[1] + negY[1]) / 2.0f;
  cal.accelBiasZ = (posZ[2] + negZ[2]) / 2.0f;

  // ----------------------------------------------------------
  // CALCULATE SCALE
  // ----------------------------------------------------------

  cal.accelScaleX =
    (2.0f * GRAVITY) /
    (posX[0] - negX[0]);

  cal.accelScaleY =
    (2.0f * GRAVITY) /
    (posY[1] - negY[1]);

  cal.accelScaleZ =
    (2.0f * GRAVITY) /
    (posZ[2] - negZ[2]);

  // ----------------------------------------------------------
  // DISPLAY RESULTS
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("============================================================");
  Serial.println("             ACCELEROMETER CALIBRATION RESULTS");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("ACCEL BIAS:");

  Serial.print("X = ");
  Serial.print(cal.accelBiasX, 8);
  Serial.println(" m/s^2");

  Serial.print("Y = ");
  Serial.print(cal.accelBiasY, 8);
  Serial.println(" m/s^2");

  Serial.print("Z = ");
  Serial.print(cal.accelBiasZ, 8);
  Serial.println(" m/s^2");

  Serial.println();
  Serial.println("ACCEL SCALE:");

  Serial.print("X = ");
  Serial.println(cal.accelScaleX, 8);

  Serial.print("Y = ");
  Serial.println(cal.accelScaleY, 8);

  Serial.print("Z = ");
  Serial.println(cal.accelScaleZ, 8);

  // ----------------------------------------------------------
  // VALIDATION
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("Validation:");

  float correctedPosX =
    (posX[0] - cal.accelBiasX) *
    cal.accelScaleX;

  float correctedNegX =
    (negX[0] - cal.accelBiasX) *
    cal.accelScaleX;

  float correctedPosY =
    (posY[1] - cal.accelBiasY) *
    cal.accelScaleY;

  float correctedNegY =
    (negY[1] - cal.accelBiasY) *
    cal.accelScaleY;

  float correctedPosZ =
    (posZ[2] - cal.accelBiasZ) *
    cal.accelScaleZ;

  float correctedNegZ =
    (negZ[2] - cal.accelBiasZ) *
    cal.accelScaleZ;

  Serial.println();

  Serial.print("+X corrected = ");
  Serial.print(correctedPosX, 5);
  Serial.println(" m/s^2");

  Serial.print("-X corrected = ");
  Serial.print(correctedNegX, 5);
  Serial.println(" m/s^2");

  Serial.print("+Y corrected = ");
  Serial.print(correctedPosY, 5);
  Serial.println(" m/s^2");

  Serial.print("-Y corrected = ");
  Serial.print(correctedNegY, 5);
  Serial.println(" m/s^2");

  Serial.print("+Z corrected = ");
  Serial.print(correctedPosZ, 5);
  Serial.println(" m/s^2");

  Serial.print("-Z corrected = ");
  Serial.print(correctedNegZ, 5);
  Serial.println(" m/s^2");

  Serial.println();

  if (cal.accelScaleX <= 0 ||
      cal.accelScaleY <= 0 ||
      cal.accelScaleZ <= 0)
  {
    Serial.println("WARNING: One or more scale values are invalid.");
    Serial.println("Check that +AX and -AX orientations were correct.");
    Serial.println("Check that +AY and -AY orientations were correct.");
    Serial.println("Check that +AZ and -AZ orientations were correct.");
  }
  else
  {
    Serial.println("Accelerometer calibration completed.");
  }
}

// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyroscope()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                  GYROSCOPE CALIBRATION");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("Place the MPU6050 in a fixed position.");
  Serial.println("DO NOT TOUCH OR MOVE IT.");
  Serial.println();
  Serial.println("Press ENTER when ready...");

  waitForUser();

  Serial.println();
  Serial.println("Settling...");
  delay(1500);

  Statistics gxStats;
  Statistics gyStats;
  Statistics gzStats;

  statsReset(gxStats);
  statsReset(gyStats);
  statsReset(gzStats);

  uint32_t intervalUs = 1000000UL / CAL_SAMPLE_HZ;

  uint32_t nextSample = micros();

  int lastPercent = -1;

  for (uint32_t i = 0; i < GYRO_CAL_SAMPLES; i++)
  {
    waitForNextSample(nextSample, intervalUs);

    SensorData s;
    readSensor(s);

    statsAdd(gxStats, s.gx);
    statsAdd(gyStats, s.gy);
    statsAdd(gzStats, s.gz);

    printProgress(i, GYRO_CAL_SAMPLES, lastPercent);
  }

  Serial.println();

  cal.gyroBiasX = statsMean(gxStats);
  cal.gyroBiasY = statsMean(gyStats);
  cal.gyroBiasZ = statsMean(gzStats);

  Serial.println();
  Serial.println("============================================================");
  Serial.println("                 GYROSCOPE CALIBRATION RESULTS");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("GYRO BIAS:");

  Serial.print("X = ");
  Serial.print(cal.gyroBiasX, 9);
  Serial.println(" rad/s");

  Serial.print("Y = ");
  Serial.print(cal.gyroBiasY, 9);
  Serial.println(" rad/s");

  Serial.print("Z = ");
  Serial.print(cal.gyroBiasZ, 9);
  Serial.println(" rad/s");

  Serial.println();

  Serial.print("X bias = ");
  Serial.print(gyroRadToDeg(cal.gyroBiasX), 6);
  Serial.println(" deg/s");

  Serial.print("Y bias = ");
  Serial.print(gyroRadToDeg(cal.gyroBiasY), 6);
  Serial.println(" deg/s");

  Serial.print("Z bias = ");
  Serial.print(gyroRadToDeg(cal.gyroBiasZ), 6);
  Serial.println(" deg/s");

  Serial.println();

  Serial.println("Stationary gyro noise:");

  Serial.print("X = ");
  Serial.print(gyroRadToDeg(statsStdDev(gxStats)), 6);
  Serial.println(" deg/s");

  Serial.print("Y = ");
  Serial.print(gyroRadToDeg(statsStdDev(gyStats)), 6);
  Serial.println(" deg/s");

  Serial.print("Z = ");
  Serial.print(gyroRadToDeg(statsStdDev(gzStats)), 6);
  Serial.println(" deg/s");

  Serial.println();
  Serial.println("Gyroscope calibration completed.");
}

// ============================================================
// BALANCE POSITION ZERO
// ============================================================

void calibrateBalanceZero()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                  BALANCE ZERO CALIBRATION");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("Now mount/hold the MPU6050 exactly as it will be");
  Serial.println("mounted on the self-balancing robot.");
  Serial.println();

  Serial.println("Place the robot in its true upright/balance position.");
  Serial.println("The wheels should be in the position where the robot");
  Serial.println("would ideally be perfectly vertical.");
  Serial.println();

  Serial.println("DO NOT MOVE IT during measurement.");
  Serial.println();
  Serial.println("Press ENTER when ready...");

  waitForUser();

  Serial.println();
  Serial.println("Settling...");
  delay(1500);

  Statistics pitchStats;
  Statistics rollStats;

  statsReset(pitchStats);
  statsReset(rollStats);

  uint32_t intervalUs = 1000000UL / CAL_SAMPLE_HZ;

  uint32_t nextSample = micros();

  int lastPercent = -1;

  for (uint32_t i = 0; i < ZERO_CAL_SAMPLES; i++)
  {
    waitForNextSample(nextSample, intervalUs);

    SensorData s;
    readSensor(s);

    float ax =
      correctAccelX(s.ax);

    float ay =
      correctAccelY(s.ay);

    float az =
      correctAccelZ(s.az);

    float pitch =
      calculatePitch(ax, ay, az);

    float roll =
      calculateRoll(ax, ay, az);

    statsAdd(pitchStats, pitch);
    statsAdd(rollStats, roll);

    printProgress(i, ZERO_CAL_SAMPLES, lastPercent);
  }

  Serial.println();

  cal.balanceZeroPitch =
    statsMean(pitchStats);

  cal.balanceZeroRoll =
    statsMean(rollStats);

  Serial.println();
  Serial.println("============================================================");
  Serial.println("                  BALANCE ZERO RESULTS");
  Serial.println("============================================================");

  Serial.println();

  Serial.print("Pitch zero = ");
  Serial.print(cal.balanceZeroPitch, 6);
  Serial.println(" degrees");

  Serial.print("Roll zero  = ");
  Serial.print(cal.balanceZeroRoll, 6);
  Serial.println(" degrees");

  Serial.println();

  Serial.print("Pitch noise = ");
  Serial.print(statsStdDev(pitchStats), 6);
  Serial.println(" degrees");

  Serial.print("Roll noise = ");
  Serial.print(statsStdDev(rollStats), 6);
  Serial.println(" degrees");

  Serial.println();

  if (BALANCE_AXIS == AXIS_PITCH)
  {
    Serial.println("Selected balancing axis: PITCH");
    Serial.println("PID angle = PITCH - PitchZero");
    Serial.println("PID rate  = corrected Gyro Y");
  }
  else
  {
    Serial.println("Selected balancing axis: ROLL");
    Serial.println("PID angle = ROLL - RollZero");
    Serial.println("PID rate  = corrected Gyro X");
  }
}

// ============================================================
// NOISE + TIMING TEST
// ============================================================

void runNoiseAndTimingTest()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("               PID SENSOR NOISE / TIMING TEST");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("Keep the robot completely stationary.");
  Serial.println("It should remain in the upright balance position.");
  Serial.println();

  Serial.println("Starting in 3 seconds...");

  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);

  Statistics angleStats;
  Statistics rateStats;
  Statistics accelAngleStats;
  Statistics dtStats;

  statsReset(angleStats);
  statsReset(rateStats);
  statsReset(accelAngleStats);
  statsReset(dtStats);

  uint32_t intervalUs =
    1000000UL / TEST_SAMPLE_HZ;

  uint32_t nextSample = micros();

  uint32_t previousMicros = micros();

  for (uint32_t i = 0; i < NOISE_TEST_SAMPLES; i++)
  {
    waitForNextSample(nextSample, intervalUs);

    uint32_t nowMicros = micros();

    float dt =
      (nowMicros - previousMicros) / 1000000.0f;

    previousMicros = nowMicros;

    SensorData s;

    readSensor(s);

    float ax =
      correctAccelX(s.ax);

    float ay =
      correctAccelY(s.ay);

    float az =
      correctAccelZ(s.az);

    float pitch =
      calculatePitch(ax, ay, az);

    float roll =
      calculateRoll(ax, ay, az);

    float gyroX =
      correctGyroX(s.gx);

    float gyroY =
      correctGyroY(s.gy);

    float balanceAngle =
      getBalanceAngle(pitch, roll);

    float balanceRate =
      getBalanceRate(gyroX, gyroY);

    float accelAngle =
      (BALANCE_AXIS == AXIS_PITCH)
        ? pitch - cal.balanceZeroPitch
        : roll - cal.balanceZeroRoll;

    statsAdd(angleStats, balanceAngle);
    statsAdd(rateStats, balanceRate);
    statsAdd(accelAngleStats, accelAngle);

    if (i > 0)
    {
      statsAdd(dtStats, dt * 1000.0f);
    }
  }

  Serial.println();
  Serial.println("TEST COMPLETE.");
  Serial.println();

  Serial.println("------------------------------------------------------------");
  Serial.println("ANGLE");
  Serial.println("------------------------------------------------------------");

  Serial.print("Mean = ");
  Serial.print(statsMean(angleStats), 6);
  Serial.println(" deg");

  Serial.print("StdDev = ");
  Serial.print(statsStdDev(angleStats), 6);
  Serial.println(" deg");

  Serial.print("Peak-to-Peak = ");
  Serial.print(statsPeakToPeak(angleStats), 6);
  Serial.println(" deg");

  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.println("GYRO RATE");
  Serial.println("------------------------------------------------------------");

  Serial.print("Mean = ");
  Serial.print(statsMean(rateStats), 6);
  Serial.println(" deg/s");

  Serial.print("StdDev = ");
  Serial.print(statsStdDev(rateStats), 6);
  Serial.println(" deg/s");

  Serial.print("Peak-to-Peak = ");
  Serial.print(statsPeakToPeak(rateStats), 6);
  Serial.println(" deg/s");

  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.println("LOOP TIMING");
  Serial.println("------------------------------------------------------------");

  Serial.print("Target frequency = ");
  Serial.print(TEST_SAMPLE_HZ);
  Serial.println(" Hz");

  Serial.print("Average dt = ");
  Serial.print(statsMean(dtStats), 4);
  Serial.println(" ms");

  Serial.print("dt StdDev = ");
  Serial.print(statsStdDev(dtStats), 4);
  Serial.println(" ms");

  float actualHz =
    1000.0f / statsMean(dtStats);

  Serial.print("Actual frequency = ");
  Serial.print(actualHz, 2);
  Serial.println(" Hz");

  Serial.println();
}

// ============================================================
// PRINT PID-READY SUMMARY
// ============================================================

void printPIDSummary()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                 PID-READY CALIBRATION DATA");
  Serial.println("============================================================");

  Serial.println();

  Serial.println("COPY THESE VALUES INTO YOUR ROBOT CODE:");
  Serial.println();

  Serial.println("// MPU6050 ACCELEROMETER");
  Serial.print("float accelBiasX = ");
  Serial.print(cal.accelBiasX, 8);
  Serial.println("f;");

  Serial.print("float accelBiasY = ");
  Serial.print(cal.accelBiasY, 8);
  Serial.println("f;");

  Serial.print("float accelBiasZ = ");
  Serial.print(cal.accelBiasZ, 8);
  Serial.println("f;");

  Serial.print("float accelScaleX = ");
  Serial.print(cal.accelScaleX, 8);
  Serial.println("f;");

  Serial.print("float accelScaleY = ");
  Serial.print(cal.accelScaleY, 8);
  Serial.println("f;");

  Serial.print("float accelScaleZ = ");
  Serial.print(cal.accelScaleZ, 8);
  Serial.println("f;");

  Serial.println();

  Serial.println("// MPU6050 GYROSCOPE");

  Serial.print("float gyroBiasX = ");
  Serial.print(cal.gyroBiasX, 9);
  Serial.println("f;");

  Serial.print("float gyroBiasY = ");
  Serial.print(cal.gyroBiasY, 9);
  Serial.println("f;");

  Serial.print("float gyroBiasZ = ");
  Serial.print(cal.gyroBiasZ, 9);
  Serial.println("f;");

  Serial.println();

  Serial.println("// BALANCE ZERO");

  Serial.print("float balanceZeroPitch = ");
  Serial.print(cal.balanceZeroPitch, 8);
  Serial.println("f;");

  Serial.print("float balanceZeroRoll = ");
  Serial.print(cal.balanceZeroRoll, 8);
  Serial.println("f;");

  Serial.println();

  Serial.println("// COMPLEMENTARY FILTER");

  Serial.print("float alpha = ");
  Serial.print(complementaryAlpha, 4);
  Serial.println("f;");

  Serial.println();

  Serial.println("// CONTROL LOOP");

  Serial.print("Target loop frequency = ");
  Serial.print(TEST_SAMPLE_HZ);
  Serial.println(" Hz");

  Serial.print("Target dt = ");
  Serial.print(1000.0f / TEST_SAMPLE_HZ, 4);
  Serial.println(" ms");

  Serial.println();

  if (BALANCE_AXIS == AXIS_PITCH)
  {
    Serial.println("BALANCING AXIS = PITCH");
    Serial.println("ANGLE INPUT = corrected accelerometer pitch");
    Serial.println("RATE INPUT  = corrected Gyro Y");
  }
  else
  {
    Serial.println("BALANCING AXIS = ROLL");
    Serial.println("ANGLE INPUT = corrected accelerometer roll");
    Serial.println("RATE INPUT  = corrected Gyro X");
  }

  Serial.println();

  Serial.println("PID relationship:");
  Serial.println();
  Serial.println("error = setpoint - angle");
  Serial.println("P = Kp * error");
  Serial.println("I = Ki * integral(error * dt)");
  Serial.println("D = Kd * derivative(error)");
  Serial.println();
  Serial.println("For a balancing robot, the gyro rate is");
  Serial.println("usually a cleaner signal for the D term than");
  Serial.println("numerically differentiating the accelerometer angle.");

  Serial.println();
}

// ============================================================
// SAVE CALIBRATION
// ============================================================

void saveCalibration()
{
  cal.magic = CAL_MAGIC;

  preferences.begin("mpu6050", false);

  size_t written =
    preferences.putBytes(
      "cal",
      &cal,
      sizeof(cal)
    );

  preferences.end();

  if (written == sizeof(cal))
  {
    Serial.println();
    Serial.println("Calibration saved to ESP32 flash.");
  }
  else
  {
    Serial.println();
    Serial.println("ERROR: Could not save calibration.");
  }
}

// ============================================================
// LOAD CALIBRATION
// ============================================================

bool loadCalibration()
{
  preferences.begin("mpu6050", true);

  size_t length =
    preferences.getBytesLength("cal");

  if (length != sizeof(cal))
  {
    preferences.end();
    return false;
  }

  preferences.getBytes(
    "cal",
    &cal,
    sizeof(cal)
  );

  preferences.end();

  if (cal.magic != CAL_MAGIC)
  {
    return false;
  }

  return true;
}

// ============================================================
// PRINT SAVED CALIBRATION
// ============================================================

void printSavedCalibration()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                  SAVED CALIBRATION");
  Serial.println("============================================================");

  Serial.println();

  Serial.print("Accel Bias X = ");
  Serial.println(cal.accelBiasX, 8);

  Serial.print("Accel Bias Y = ");
  Serial.println(cal.accelBiasY, 8);

  Serial.print("Accel Bias Z = ");
  Serial.println(cal.accelBiasZ, 8);

  Serial.println();

  Serial.print("Accel Scale X = ");
  Serial.println(cal.accelScaleX, 8);

  Serial.print("Accel Scale Y = ");
  Serial.println(cal.accelScaleY, 8);

  Serial.print("Accel Scale Z = ");
  Serial.println(cal.accelScaleZ, 8);

  Serial.println();

  Serial.print("Gyro Bias X = ");
  Serial.print(gyroRadToDeg(cal.gyroBiasX), 6);
  Serial.println(" deg/s");

  Serial.print("Gyro Bias Y = ");
  Serial.print(gyroRadToDeg(cal.gyroBiasY), 6);
  Serial.println(" deg/s");

  Serial.print("Gyro Bias Z = ");
  Serial.print(gyroRadToDeg(cal.gyroBiasZ), 6);
  Serial.println(" deg/s");

  Serial.println();

  Serial.print("Pitch Zero = ");
  Serial.println(cal.balanceZeroPitch, 6);

  Serial.print("Roll Zero = ");
  Serial.println(cal.balanceZeroRoll, 6);
}

// ============================================================
// FULL CALIBRATION
// ============================================================

void runFullCalibration()
{
  Serial.println();
  Serial.println("############################################################");
  Serial.println("#                   FULL CALIBRATION                       #");
  Serial.println("############################################################");

  Serial.println();

  calibrateAccelerometer();

  delay(1000);

  calibrateGyroscope();

  delay(1000);

  calibrateBalanceZero();

  delay(1000);

  saveCalibration();

  delay(500);

  runNoiseAndTimingTest();

  printPIDSummary();
}

// ============================================================
// LIVE SENSOR MODE
// ============================================================

void liveSensorMode()
{
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                    LIVE PID SENSOR DATA");
  Serial.println("============================================================");

  Serial.println();
  Serial.println("Output every 100 ms.");
  Serial.println("Press 'C' to start full calibration.");
  Serial.println();

  float filteredAngle = 0.0f;

  bool firstReading = true;

  uint32_t previousMicros = micros();

  uint32_t lastPrint = millis();

  // Target control loop
  const uint32_t intervalUs =
    1000000UL / TEST_SAMPLE_HZ;

  uint32_t nextSample = micros();

  while (true)
  {
    // --------------------------------------------------------
    // Serial command
    // --------------------------------------------------------

    if (Serial.available())
    {
      char command = Serial.read();

      if (command == 'C' || command == 'c')
      {
        Serial.println();
        Serial.println("Calibration command received.");

        runFullCalibration();

        filteredAngle = 0.0f;
        firstReading = true;

        previousMicros = micros();
        nextSample = micros();

        clearSerialBuffer();
      }
    }

    // --------------------------------------------------------
    // Fixed-rate sampling
    // --------------------------------------------------------

    waitForNextSample(nextSample, intervalUs);

    uint32_t now = micros();

    float dt =
      (now - previousMicros) / 1000000.0f;

    previousMicros = now;

    SensorData s;

    readSensor(s);

    // --------------------------------------------------------
    // Calibrate ACCEL
    // --------------------------------------------------------

    float ax =
      correctAccelX(s.ax);

    float ay =
      correctAccelY(s.ay);

    float az =
      correctAccelZ(s.az);

    // --------------------------------------------------------
    // Calculate angles
    // --------------------------------------------------------

    float pitch =
      calculatePitch(ax, ay, az);

    float roll =
      calculateRoll(ax, ay, az);

    // --------------------------------------------------------
    // Correct gyro
    // --------------------------------------------------------

    float gyroX =
      correctGyroX(s.gx);

    float gyroY =
      correctGyroY(s.gy);

    // --------------------------------------------------------
    // Select balancing axis
    // --------------------------------------------------------

    float accelAngle =
      getBalanceAngle(pitch, roll);

    float gyroRate =
      getBalanceRate(gyroX, gyroY);

    // --------------------------------------------------------
    // Complementary filter
    // --------------------------------------------------------

    if (firstReading)
    {
      filteredAngle = accelAngle;
      firstReading = false;
    }
    else
    {
      filteredAngle =
        complementaryAlpha *
        (filteredAngle + gyroRate * dt)
        +
        (1.0f - complementaryAlpha) *
        accelAngle;
    }

    // --------------------------------------------------------
    // Serial display
    // --------------------------------------------------------

    if (millis() - lastPrint >= 100)
    {
      lastPrint = millis();

      Serial.print("ANGLE=");
      Serial.print(filteredAngle, 3);

      Serial.print(", ACCEL=");
      Serial.print(accelAngle, 3);

      Serial.print(", RATE=");
      Serial.print(gyroRate, 3);

      Serial.print(", PITCH=");
      Serial.print(pitch, 3);

      Serial.print(", ROLL=");
      Serial.print(roll, 3);

      Serial.print(", DT=");
      Serial.print(dt * 1000.0f, 3);

      Serial.println(" ms");
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println();
  Serial.println("ESP32 MPU6050 Calibration System");
  Serial.println();

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);

  // ----------------------------------------------------------
  // Detect MPU6050
  // ----------------------------------------------------------

  if (!detectMPU())
  {
    Serial.println();
    Serial.println("Check:");
    Serial.println("  SDA -> GPIO 21");
    Serial.println("  SCL -> GPIO 22");
    Serial.println("  VCC -> 3.3V");
    Serial.println("  GND -> GND");

    while (true)
    {
      delay(1000);
    }
  }

  // ----------------------------------------------------------
  // Initialize
  // ----------------------------------------------------------

  if (!initializeMPU())
  {
    while (true)
    {
      delay(1000);
    }
  }

  printHeader();

  // ----------------------------------------------------------
  // Calibration storage
  // ----------------------------------------------------------

  if (loadCalibration())
  {
    Serial.println("A valid calibration is stored in ESP32 flash.");

    printSavedCalibration();

    Serial.println();
    Serial.println("Startup options:");
    Serial.println();
    Serial.println("  C = Full calibration");
    Serial.println("  Z = Recalibrate balance zero only");
    Serial.println("  ENTER = Use saved calibration");
    Serial.println();

    Serial.println("Waiting 5 seconds...");

    uint32_t startTime = millis();

    bool commandReceived = false;

    while (millis() - startTime < 5000)
    {
      if (Serial.available())
      {
        char command = Serial.read();

        if (command == 'C' || command == 'c')
        {
          commandReceived = true;

          runFullCalibration();
          break;
        }

        if (command == 'Z' || command == 'z')
        {
          commandReceived = true;

          calibrateBalanceZero();
          saveCalibration();

          runNoiseAndTimingTest();

          printPIDSummary();

          break;
        }

        if (command == '\n' ||
            command == '\r')
        {
          commandReceived = true;
          break;
        }
      }

      delay(10);
    }

    if (!commandReceived)
    {
      Serial.println("Using saved calibration.");
    }
  }
  else
  {
    Serial.println();
    Serial.println("No valid calibration found.");
    Serial.println("Starting first-time calibration.");

    delay(1500);

    runFullCalibration();
  }

  // ----------------------------------------------------------
  // Final summary
  // ----------------------------------------------------------

  printPIDSummary();

  delay(1000);

  // ----------------------------------------------------------
  // Live mode
  // ----------------------------------------------------------

  liveSensorMode();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // Everything runs inside liveSensorMode()
}