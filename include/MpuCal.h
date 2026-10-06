#pragma once
/*
 * MpuCal.h - Shared MPU6050 calibration helpers

 * WHAT IT DOES:
    Defines the calibration record, flash load/save, sensor init and
    angle helpers shared by the calibration tool and the PID firmware.

 * HOW TO USE:
    Copy into the include/ folder of the balancing project, then call
    mpuBegin(), mpuCalLoad() and mpuQuickGyroZero() in setup().

 * NOTE:
    Angle = accelPitchDeg() - balanceAngle; positive means leaning forward.
    Keep the sensor range and filter settings identical in both projects.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>

// Hardware + sensor configuration
#define MPU_SDA_PIN      21
#define MPU_SCL_PIN      22

#define MPU_ACCEL_RANGE  MPU6050_RANGE_2_G
#define MPU_GYRO_RANGE   MPU6050_RANGE_500_DEG
#define MPU_DLPF         MPU6050_BAND_44_HZ

#define MPU_GRAVITY      9.80665f

// Calibration record stored in flash (NVS, namespace "mpu6050", key "cal")
#define MPU_CAL_MAGIC    0x6050CA11UL
#define MPU_CAL_VERSION  2

struct MpuCalibration
{
  uint32_t magic;
  uint32_t version;
  float accelBias[3];    // m/s^2   (X, Y, Z)
  float accelScale[3];   // unitless multiplier
  float gyroBias[3];     // rad/s   (X, Y, Z)
  float balanceAngle;    // degrees, accelerometer pitch at which the robot balances
};

inline void mpuCalDefaults(MpuCalibration &c)
{
  memset(&c, 0, sizeof(c));
  c.magic = MPU_CAL_MAGIC;
  c.version = MPU_CAL_VERSION;
  for (int i = 0; i < 3; i++) c.accelScale[i] = 1.0f;
}

inline bool mpuCalLoad(MpuCalibration &c)
{
  Preferences p;
  if (!p.begin("mpu6050", false)) return false;

  bool ok = false;
  if (p.getBytesLength("cal") == sizeof(MpuCalibration))
  {
    MpuCalibration tmp;
    p.getBytes("cal", &tmp, sizeof(tmp));
    if (tmp.magic == MPU_CAL_MAGIC && tmp.version == MPU_CAL_VERSION)
    {
      c = tmp;
      ok = true;
    }
  }
  p.end();
  return ok;
}

inline bool mpuCalSave(MpuCalibration &c)
{
  c.magic = MPU_CAL_MAGIC;
  c.version = MPU_CAL_VERSION;

  Preferences p;
  if (!p.begin("mpu6050", false)) return false;
  size_t n = p.putBytes("cal", &c, sizeof(c));
  p.end();
  return n == sizeof(c);
}

inline void mpuCalErase()
{
  Preferences p;
  if (p.begin("mpu6050", false))
  {
    p.remove("cal");
    p.end();
  }
}

// Sensor init (tries 0x68, then 0x69)
inline bool mpuBegin(Adafruit_MPU6050 &mpu)
{
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);

  if (!mpu.begin(0x68, &Wire) && !mpu.begin(0x69, &Wire)) return false;

  mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
  mpu.setGyroRange(MPU_GYRO_RANGE);
  mpu.setFilterBandwidth(MPU_DLPF);

  Wire.setClock(400000);   // after begin(), so nothing can reset it
  delay(100);
  return true;
}

// Helpers
inline float wrap180(float a)
{
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

inline float accelPitchDeg(float ax, float ay, float az)
{
  float r = sqrtf(ay * ay + az * az);
  if (az < 0.0f) r = -r;
  return atan2f(-ax, r) * RAD_TO_DEG;
}

inline float accelRollDeg(float ay, float az)
{
  return atan2f(ay, az) * RAD_TO_DEG;
}

// Apply bias + scale to raw accelerometer values (m/s^2)
inline void mpuCorrectAccel(const MpuCalibration &c, float &ax, float &ay, float &az)
{
  ax = (ax - c.accelBias[0]) * c.accelScale[0];
  ay = (ay - c.accelBias[1]) * c.accelScale[1];
  az = (az - c.accelBias[2]) * c.accelScale[2];
}

inline void mpuQuickGyroZero(Adafruit_MPU6050 &mpu, MpuCalibration &c, uint16_t samples = 500)
{
  double sum[3] = {0.0, 0.0, 0.0};

  for (uint16_t i = 0; i < samples; i++)
  {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum[0] += g.gyro.x;
    sum[1] += g.gyro.y;
    sum[2] += g.gyro.z;
    delay(2);
  }

  for (int k = 0; k < 3; k++) c.gyroBias[k] = (float)(sum[k] / samples);
}
