/*
 * MPU6050 Calibration Tool (ESP32 self-balancing robot)

 * WHAT IT DOES:
    Calibrates the accelerometer (6-face) and gyro, finds the robot's
    balance point, saves everything to flash, then streams live angle data.

 * HOW TO USE:
    Open the PlatformIO serial monitor (115200). Send F for full calibration
    and follow the prompts. Send M in live mode to return to the menu.

 * NOTE:
    Wiring: SDA = GPIO 21, SCL = GPIO 22, VCC = 3.3V.
    Mounting: X forward, Y toward the wheels, Z up.
*/

#include <Arduino.h>
#include "MpuCal.h"

// Settings
#define SAMPLE_HZ        250
#define FACE_SAMPLES     1000
#define GYRO_SAMPLES     2000
#define BALANCE_SAMPLES  1000
#define NOISE_SAMPLES    2000
#define WARMUP_SECONDS   30
#define LIVE_PRINT_MS    100

static const float ALPHA = 0.98f;
static const float G     = MPU_GRAVITY;

Adafruit_MPU6050 mpu;
MpuCalibration   cal;

// Sensor sample + running statistics
struct Sample
{
  float a[3];     // m/s^2
  float g[3];     // rad/s
  float temp;     // deg C
};

static void readSample(Sample &s)
{
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  s.a[0] = a.acceleration.x;
  s.a[1] = a.acceleration.y;
  s.a[2] = a.acceleration.z;

  s.g[0] = g.gyro.x;
  s.g[1] = g.gyro.y;
  s.g[2] = g.gyro.z;

  s.temp = t.temperature;
}

struct Channel
{
  double mean = 0.0;
  double m2 = 0.0;
  uint32_t n = 0;
  float mn = 1e30f;
  float mx = -1e30f;

  void add(double x)
  {
    n++;
    double d = x - mean;
    mean += d / n;
    m2 += d * (x - mean);
    if (x < mn) mn = (float)x;
    if (x > mx) mx = (float)x;
  }

  double sd() const { return (n > 1) ? sqrt(m2 / (n - 1)) : 0.0; }
};

struct Block { Channel ch[6]; };

// Fixed-rate sampling
static inline void waitNext(uint32_t &next, uint32_t interval)
{
  next += interval;
  while ((int32_t)(micros() - next) < 0) { }

  // fell behind by more than one period (e.g. slow print) -> resync
  if ((int32_t)(micros() - next) > (int32_t)interval) next = micros();
}

static void collect(Block &b, uint16_t n)
{
  const uint32_t interval = 1000000UL / SAMPLE_HZ;
  uint32_t next = micros();
  int lastPct = -1;

  for (uint16_t i = 0; i < n; i++)
  {
    waitNext(next, interval);

    Sample s;
    readSample(s);

    for (int k = 0; k < 3; k++)
    {
      b.ch[k].add(s.a[k]);
      b.ch[k + 3].add(s.g[k]);
    }

    int pct = ((i + 1) * 100) / n;
    if (pct / 10 != lastPct / 10)
    {
      Serial.printf("%d%% ", pct);
      lastPct = pct;
    }
  }
  Serial.println();
}

// Serial helpers
static void flushInput()
{
  while (Serial.available()) Serial.read();
}

static String readLine()
{
  String s;
  for (;;)
  {
    if (!Serial.available())
    {
      delay(2);
      continue;
    }

    char c = Serial.read();
    if (c == '\r' || c == '\n')
    {
      delay(20);
      while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n')) Serial.read();
      s.trim();
      return s;
    }
    s += c;
  }
}

static String ask(const char *msg)
{
  Serial.println(msg);
  flushInput();
  return readLine();
}

static void saveCal()
{
  Serial.println(mpuCalSave(cal) ? "Calibration saved to flash." : "ERROR: could not save calibration!");
}

// Warm-up
static void warmUp(uint32_t seconds)
{
  Serial.printf("\nWarming the sensor up for %lu s (readings drift with chip temperature).\n", (unsigned long)seconds);
  Serial.println("Send ENTER to skip.");
  flushInput();

  uint32_t t0 = millis();
  uint32_t lastPrint = t0 - 5000;

  while (millis() - t0 < seconds * 1000UL)
  {
    if (Serial.available()) break;

    if (millis() - lastPrint >= 5000)
    {
      lastPrint = millis();
      Sample s;
      readSample(s);
      Serial.printf("  %2lu s   chip temperature %.1f C\n", (unsigned long)((millis() - t0) / 1000), s.temp);
    }
    delay(20);
  }

  delay(30);
  flushInput();
}

// 1) SIX-FACE ACCELEROMETER CALIBRATION
static bool collectFace(const char *label, int axis, int sign, float out[3])
{
  for (;;)
  {
    Serial.printf("\n--- %s ---\n", label);
    ask("Place the sensor like this and keep it COMPLETELY STILL, then press ENTER.");
    delay(1000);

    Block b;
    collect(b, FACE_SAMPLES);

    float m[3], sd[3];
    for (int k = 0; k < 3; k++)
    {
      m[k]  = (float)b.ch[k].mean;
      sd[k] = (float)b.ch[k].sd();
    }

    Serial.printf("mean  : X=%8.4f  Y=%8.4f  Z=%8.4f  m/s^2\n", m[0], m[1], m[2]);
    Serial.printf("noise : X=%8.4f  Y=%8.4f  Z=%8.4f  m/s^2\n", sd[0], sd[1], sd[2]);

    bool bad = false;

    if (sign * m[axis] < 0.90f * G)
    {
      Serial.println("!! The expected axis does not read ~1 g: wrong face.");
      bad = true;
    }

    for (int k = 0; k < 3; k++)
    {
      if (k != axis && fabsf(m[k]) > 0.15f * G)
      {
        Serial.println("!! Sensor is not level on this face (more than ~8 deg tilt).");
        bad = true;
        break;
      }
    }

    if (fmaxf(sd[0], fmaxf(sd[1], sd[2])) > 0.10f)
    {
      Serial.println("!! Movement / vibration detected.");
      bad = true;
    }

    if (bad)
    {
      String r = ask("Press ENTER to repeat this face (or type A + ENTER to accept it anyway).");
      if (!r.equalsIgnoreCase("A")) continue;
    }

    for (int k = 0; k < 3; k++) out[k] = m[k];
    return true;
  }
}

static bool calibrateAccel()
{
  Serial.println("\n============ 6-FACE ACCELEROMETER CALIBRATION ============");
  Serial.println("Use a flat, level, vibration-free surface (a table).");
  Serial.println("Use the X / Y / Z arrows printed on the MPU6050 board.");
  Serial.println("Only gravity matters here, so the board can be loose or already mounted.");

  static const char *names[6] =
  {
    "+X UP : X arrow pointing to the CEILING",
    "-X UP : X arrow pointing to the FLOOR",
    "+Y UP : Y arrow pointing to the CEILING",
    "-Y UP : Y arrow pointing to the FLOOR",
    "+Z UP : Z arrow pointing to the CEILING (board flat, components up)",
    "-Z UP : Z arrow pointing to the FLOOR (board flat, components down)"
  };

  float face[6][3];
  for (int f = 0; f < 6; f++)
  {
    collectFace(names[f], f / 2, (f % 2 == 0) ? +1 : -1, face[f]);
  }

  float bias[3], scale[3];

  for (int i = 0; i < 3; i++)
  {
    float pos = face[2 * i][i];
    float neg = face[2 * i + 1][i];

    if (pos - neg < 1.6f * G)
    {
      Serial.printf("ERROR: axis %c span is only %.2f m/s^2 (expected ~%.2f). Check faces and redo.\n",
                    "XYZ"[i], pos - neg, 2.0f * G);
      return false;
    }

    bias[i]  = 0.5f * (pos + neg);
    scale[i] = (2.0f * G) / (pos - neg);
  }

  float worst = 0.0f;
  for (int f = 0; f < 6; f++)
  {
    for (int k = 0; k < 3; k++)
    {
      if (k == f / 2) continue;
      float v = (face[f][k] - bias[k]) * scale[k];
      worst = fmaxf(worst, fabsf(v));
    }
  }

  memcpy(cal.accelBias,  bias,  sizeof(bias));
  memcpy(cal.accelScale, scale, sizeof(scale));

  Serial.println("\n---------------- RESULT ----------------");
  Serial.printf("Bias  (m/s^2): X=%9.5f  Y=%9.5f  Z=%9.5f\n", bias[0], bias[1], bias[2]);
  Serial.printf("Scale        : X=%9.5f  Y=%9.5f  Z=%9.5f\n", scale[0], scale[1], scale[2]);
  Serial.printf("Worst off-axis residual: %.3f m/s^2 (about %.2f deg of placement error)\n", worst, asinf(fminf(1.0f, worst / G)) * RAD_TO_DEG);

  if (worst > 0.30f)
  {
    Serial.println("WARNING: faces were not very level / board axes not orthogonal. Consider redoing it.");
  }

  for (int i = 0; i < 3; i++)
  {
    if (scale[i] < 0.9f || scale[i] > 1.1f)
    {
      Serial.println("WARNING: a scale factor is more than 10% from 1.0 - something looks wrong.");
      break;
    }
  }

  saveCal();
  return true;
}

// 2) GYRO BIAS
static bool calibrateGyro()
{
  Serial.println("\n================ GYROSCOPE BIAS CALIBRATION ================");

  for (;;)
  {
    ask("Put the robot down on a solid surface. DO NOT TOUCH IT. Press ENTER.");
    delay(1500);

    Block h1, h2;
    collect(h1, GYRO_SAMPLES / 2);
    collect(h2, GYRO_SAMPLES / 2);

    float bias[3];
    float worstSd = 0.0f;
    float worstDrift = 0.0f;

    for (int k = 0; k < 3; k++)
    {
      double m1 = h1.ch[3 + k].mean;
      double m2 = h2.ch[3 + k].mean;

      bias[k] = (float)(0.5 * (m1 + m2));
      worstDrift = fmaxf(worstDrift, fabsf((float)(m1 - m2)) * RAD_TO_DEG);
      worstSd = fmaxf(worstSd, (float)fmax(h1.ch[3 + k].sd(), h2.ch[3 + k].sd()) * RAD_TO_DEG);
    }

    Serial.printf("Gyro bias: X=%.4f  Y=%.4f  Z=%.4f deg/s\n",  bias[0] * RAD_TO_DEG, bias[1] * RAD_TO_DEG, bias[2] * RAD_TO_DEG);
    Serial.printf("Noise (worst axis): %.4f deg/s | change between 1st and 2nd half: %.4f deg/s\n", worstSd, worstDrift);

    if (worstSd > 0.5f || worstDrift > 0.25f)
    {
      Serial.println("!! The robot moved (or the sensor is still warming up).");
      String r = ask("ENTER = redo, A = accept anyway");
      if (!r.equalsIgnoreCase("A")) continue;
    }

    memcpy(cal.gyroBias, bias, sizeof(bias));
    saveCal();
    return true;
  }
}

// 3) BALANCE POINT
static void measureStatic(float v[3], float &sdMax)
{
  Block b;
  collect(b, BALANCE_SAMPLES);

  sdMax = 0.0f;
  for (int k = 0; k < 3; k++)
  {
    v[k] = ((float)b.ch[k].mean - cal.accelBias[k]) * cal.accelScale[k];
    sdMax = fmaxf(sdMax, (float)b.ch[k].sd() * cal.accelScale[k]);
  }
}

// METHOD 1: pendulum.
static bool balanceByHanging(float &result)
{
  Serial.println("\nMETHOD 1 - HANG (most accurate)");
  Serial.println("Suspend the robot so it can swing freely about the WHEEL AXLE, motors unpowered:");
  Serial.println("e.g. remove the wheels and rest the axle/motor shafts on two supports, or hang");
  Serial.println("it from the axle. The body must hang with the wheels' axis horizontal.");
  Serial.println("You will swing it gently forward, let it settle, then backward, let it settle.");

  float off[2];
  static const char *prompts[2] =
  {
    "Swing the robot FORWARD about 20-30 deg, release, and let it settle completely (~15 s). Then press ENTER.",
    "Now swing it BACKWARD about 20-30 deg, release, and let it settle completely (~15 s). Then press ENTER."
  };

  for (int i = 0; i < 2; i++)
  {
    for (;;)
    {
      ask(prompts[i]);

      float v[3], sd;
      measureStatic(v, sd);

      float ang = accelPitchDeg(v[0], v[1], v[2]);

      Serial.printf("ax=%.3f ay=%.3f az=%.3f m/s^2 | noise %.3f | raw rest angle %.3f deg\n", v[0], v[1], v[2], sd, ang);

      if (v[2] > -0.5f * G)
      {
        Serial.println("!! Z should point roughly DOWN while hanging (robot upside-down relative to balancing). Check the setup.");
        continue;
      }

      if (fabsf(v[1]) > 0.15f * G)
      {
        Serial.println("!! The robot is rolled sideways: the wheel axle should be parallel to the sensor Y axis. Repeat.");
        continue;
      }

      if (sd > 0.15f)
      {
        Serial.println("!! Still swinging. Wait until it is completely still and repeat.");
        continue;
      }

      off[i] = wrap180(ang - 180.0f);
      break;
    }
  }

  float spread = fabsf(off[0] - off[1]);
  result = 0.5f * (off[0] + off[1]);

  Serial.printf("\nFrom-front estimate: %.3f deg | from-back estimate: %.3f deg | friction spread: %.3f deg\n", off[0], off[1], spread);

  if (spread > 1.5f)
  {
    Serial.println("WARNING: large spread = sticky bearings/gearbox. The average is still the best estimate, but");
    Serial.println("         consider reducing friction (free wheels/shafts) and repeating.");
  }

  if (fabsf(result) > 25.0f)
  {
    Serial.println("WARNING: balance angle is more than 25 deg from the sensor's vertical - is the board mounted as described?");
  }

  return true;
}

// METHOD 2: hold it by hand at the angle that "feels" balanced.
static bool balanceByHolding(float &result)
{
  Serial.println("\nMETHOD 2 - HOLD (quick, typically good to a degree or two)");
  Serial.println("Stand the robot on its wheels (motors unpowered). Hold it lightly with two fingers at the");
  Serial.println("angle where it neither wants to fall forward nor backward, then keep it perfectly still.");

  for (;;)
  {
    ask("Press ENTER to measure.");

    float v[3], sd;
    measureStatic(v, sd);

    float ang = accelPitchDeg(v[0], v[1], v[2]);
    Serial.printf("ax=%.3f ay=%.3f az=%.3f m/s^2 | noise %.3f | angle %.3f deg\n", v[0], v[1], v[2], sd, ang);

    if (v[2] < 0.5f * G || sd > 0.15f)
    {
      Serial.println("!! Robot not upright or not still. Repeat.");
      continue;
    }

    result = ang;
    return true;
  }
}

static bool calibrateBalance()
{
  Serial.println("\n================ BALANCE-POINT CALIBRATION ================");
  Serial.println("Needs a valid accelerometer calibration (run A first if unsure).");
  Serial.println("  1 = HANG  method (most accurate, recommended)");
  Serial.println("  2 = HOLD  method (quick)");

  String m = ask("Choose 1 or 2 (ENTER = 1):");

  float angle = 0.0f;
  bool ok = (m == "2") ? balanceByHolding(angle) : balanceByHanging(angle);
  if (!ok) return false;

  cal.balanceAngle = angle;

  Serial.printf("\nBALANCE ANGLE = %.4f deg\n", angle);
  Serial.println("PID angle = accelPitch - balanceAngle   (0 = balanced, positive = leaning forward)");

  saveCal();
  return true;
}

// Printing
static void printCal()
{
  Serial.println("\n==================== SAVED CALIBRATION ====================");
  Serial.printf("Accel bias  (m/s^2): %10.6f %10.6f %10.6f\n", cal.accelBias[0], cal.accelBias[1], cal.accelBias[2]);
  Serial.printf("Accel scale        : %10.6f %10.6f %10.6f\n", cal.accelScale[0], cal.accelScale[1], cal.accelScale[2]);
  Serial.printf("Gyro bias  (deg/s) : %10.6f %10.6f %10.6f\n", cal.gyroBias[0] * RAD_TO_DEG, cal.gyroBias[1] * RAD_TO_DEG, cal.gyroBias[2] * RAD_TO_DEG);
  Serial.printf("Balance angle (deg): %10.6f\n", cal.balanceAngle);
  Serial.println("Sensor config      : accel +/-2 g, gyro +/-500 deg/s, DLPF 44 Hz");

  Serial.println("\n// backup constants:");
  Serial.printf("float accelBias[3]  = {%.8ff, %.8ff, %.8ff};\n", cal.accelBias[0], cal.accelBias[1], cal.accelBias[2]);
  Serial.printf("float accelScale[3] = {%.8ff, %.8ff, %.8ff};\n", cal.accelScale[0], cal.accelScale[1], cal.accelScale[2]);
  Serial.printf("float gyroBias[3]   = {%.9ff, %.9ff, %.9ff}; // rad/s\n", cal.gyroBias[0], cal.gyroBias[1], cal.gyroBias[2]);
  Serial.printf("float balanceAngle  = %.6ff;\n", cal.balanceAngle);
}

// Noise + timing test
static void noiseTest()
{
  Serial.println("\n================ NOISE / TIMING TEST ================");

  if (ask("Hold/stand the robot still, roughly upright. ENTER = start, S = skip.").equalsIgnoreCase("S")) return;
  delay(1500);

  Channel filt, acc, rate, dtMs;

  const uint32_t interval = 1000000UL / SAMPLE_HZ;
  uint32_t next = micros();
  uint32_t prev = next;

  float angle = 0.0f;
  bool first = true;

  for (uint16_t i = 0; i < NOISE_SAMPLES; i++)
  {
    waitNext(next, interval);

    uint32_t now = micros();
    float dt = (now - prev) * 1e-6f;
    prev = now;

    Sample s;
    readSample(s);

    float ax = s.a[0], ay = s.a[1], az = s.a[2];
    mpuCorrectAccel(cal, ax, ay, az);

    float accAngle = wrap180(accelPitchDeg(ax, ay, az) - cal.balanceAngle);
    float r = (s.g[1] - cal.gyroBias[1]) * RAD_TO_DEG;

    if (first)
    {
      angle = accAngle;
      first = false;
    }
    else
    {
      angle = ALPHA * (angle + r * dt) + (1.0f - ALPHA) * accAngle;

      filt.add(angle);
      acc.add(accAngle);
      rate.add(r);
      dtMs.add(dt * 1000.0f);
    }
  }

  float dtAvg = (float)dtMs.mean;

  Serial.println();
  Serial.printf("Angle (filtered)   : mean %.3f deg | std %.4f deg | peak-to-peak %.4f deg\n", filt.mean, filt.sd(), filt.mx - filt.mn);
  Serial.printf("Angle (accel only) : std %.4f deg | peak-to-peak %.4f deg\n", acc.sd(), acc.mx - acc.mn);
  Serial.printf("Gyro rate          : mean %.4f deg/s (should be ~0) | std %.4f deg/s\n", rate.mean, rate.sd());
  Serial.printf("Loop timing        : target %d Hz | actual %.1f Hz | dt avg %.4f ms | std %.4f ms | min %.3f | max %.3f ms\n", SAMPLE_HZ, 1000.0f / dtAvg, dtAvg, dtMs.sd(), dtMs.mn, dtMs.mx);
  Serial.printf("Complementary filter alpha %.3f -> time constant about %.3f s\n", ALPHA, ALPHA * (1.0f / SAMPLE_HZ) / (1.0f - ALPHA));
}

// Live data
static void liveMode()
{
  Serial.println("\n==================== LIVE SENSOR DATA ====================");
  Serial.println("ANGLE = filtered balance angle (0 = balanced, + = leaning forward)");
  Serial.println("ACCEL = accelerometer-only angle, RATE = gyro rate (deg/s)");
  Serial.println("Tilt the robot forward: ANGLE and RATE must both turn positive.");
  Serial.println("Send M (+ENTER) to return to the menu.\n");

  flushInput();

  const uint32_t interval = 1000000UL / SAMPLE_HZ;
  uint32_t next = micros();
  uint32_t prev = next;
  uint32_t lastPrint = millis();

  float angle = 0.0f;
  bool first = true;

  for (;;)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      if (c == 'm' || c == 'M')
      {
        delay(30);
        flushInput();
        return;
      }
    }

    waitNext(next, interval);

    uint32_t now = micros();
    float dt = (now - prev) * 1e-6f;
    prev = now;

    Sample s;
    readSample(s);

    float ax = s.a[0], ay = s.a[1], az = s.a[2];
    mpuCorrectAccel(cal, ax, ay, az);

    float pitch = accelPitchDeg(ax, ay, az);
    float roll  = accelRollDeg(ay, az);

    float accAngle = wrap180(pitch - cal.balanceAngle);
    float rate     = (s.g[1] - cal.gyroBias[1]) * RAD_TO_DEG;

    if (first)
    {
      angle = accAngle;
      first = false;
    }
    else
    {
      angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * accAngle;
    }

    if (millis() - lastPrint >= LIVE_PRINT_MS)
    {
      lastPrint = millis();
      Serial.printf("ANGLE=%8.3f  ACCEL=%8.3f  RATE=%9.3f  PITCH=%8.3f  ROLL=%8.3f  DT=%.3f ms\n",  angle, accAngle, rate, pitch, roll, dt * 1000.0f);
    }
  }
}

// Full calibration + menu
static void runFull()
{
  Serial.println("\n############### FULL CALIBRATION ###############");

  warmUp(WARMUP_SECONDS);

  if (!calibrateAccel())   return;
  if (!calibrateGyro())    return;
  if (!calibrateBalance()) return;

  printCal();
  noiseTest();
}

static void printMenu()
{
  Serial.println("\n======================== MENU ========================");
  Serial.println("  F  Full calibration (accel 6-face -> gyro -> balance point), then live data");
  Serial.println("  A  Accelerometer 6-face only      (re-run B afterwards)");
  Serial.println("  G  Gyro bias only");
  Serial.println("  B  Balance point only");
  Serial.println("  N  Noise / timing test");
  Serial.println("  P  Print saved calibration");
  Serial.println("  L  Live data");
  Serial.println("  E  Erase calibration from flash");
}

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n\nESP32 + MPU6050 self-balancing robot calibration tool");

  if (!mpuBegin(mpu))
  {
    Serial.println("\nERROR: MPU6050 not found / init failed. Check:");
    Serial.println("  VCC -> 3.3V, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22");
    Serial.println("  (some clone boards report a different WHO_AM_I and are rejected by the library)");
    while (true) delay(1000);
  }

  Serial.println("MPU6050 initialised.");

  if (!mpuCalLoad(cal))
  {
    mpuCalDefaults(cal);
    Serial.println("\nNo valid calibration in flash -> starting full calibration.");
    runFull();
    liveMode();
    return;
  }

  printCal();
  Serial.println("\nSaved calibration loaded. Live data starts in 5 s - send any text for the menu.");

  flushInput();
  uint32_t t0 = millis();
  while (millis() - t0 < 5000)
  {
    if (Serial.available())
    {
      delay(30);
      flushInput();
      return;               // loop() shows the menu
    }
    delay(10);
  }

  liveMode();
}

void loop()
{
  printMenu();

  for (;;)
  {
    String c = ask("\nCommand:");
    c.toUpperCase();

    if (c == "F")
    {
      runFull();
      liveMode();
      return;
    }
    else if (c == "A") calibrateAccel();
    else if (c == "G") calibrateGyro();
    else if (c == "B") calibrateBalance();
    else if (c == "N") noiseTest();
    else if (c == "P") printCal();
    else if (c == "L")
    {
      liveMode();
      return;
    }
    else if (c == "E")
    {
      if (ask("Type YES to erase the stored calibration:").equalsIgnoreCase("YES"))
      {
        mpuCalErase();
        mpuCalDefaults(cal);
        Serial.println("Erased.");
      }
    }
    else
    {
      printMenu();
    }
  }
}