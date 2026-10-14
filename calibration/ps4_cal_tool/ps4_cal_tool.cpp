/*
 * PS4 Calibration (ESP32 self-balancing robot)

 * WHAT IT DOES:
    Calibrates the PS4 controller sticks and saves the result to flash (Ps4Cal.h) for the drive firmware. It measures, per stick axis:
      - centre (rest position) and noise, from which the deadzone is set
      - full travel, both directions
      - which sign means "forward" (left stick) and "right" (right stick)
    A stick that rests 5 counts off-centre would otherwise read as a constant lean command, and the robot would creep away. That is why this matters.

 * HOW TO USE:
    1. Make sure Ps4Cal.h is in include/. This sketch needs no robot hardware beyond the ESP32, so the motors can stay unpowered.
    2. Upload, open the serial monitor (115200). It prints the ESP32 Bluetooth address. Pair the controller to that address (SixaxisPairTool), then press the PS button.
    3. Follow the prompts. X advances, or send n over serial.
    4. In the live check, press Options to save (or send s).

 * STEPS:
    1 CENTRE - hands off both sticks, press X. Captures 1.5 s.
    2 RANGE - press X, then roll both sticks around the full edge for 8 s.
    3 FORWARD - push the LEFT stick fully forward and hold (auto-detected).
    4 RIGHT - push the RIGHT stick fully right and hold (auto-detected).
    5 LIVE CHECK - prints FORWARD / STEER as -1..+1. Options = save, Share = redo.

 * SERIAL (115200, no ENTER needed):
       n - next   s - save   r - restart   e - erase saved calibration

 * LIGHTBAR:
    blue = waiting for you, orange = capturing, purple = direction, green = live check

 * NOTE:
    Needs the PS4-esp32 (PS4Controller) library, i.e. Classic Bluetooth on a plain ESP32 with Arduino-ESP32 core 2.x.
*/

#include <Arduino.h>
#include <esp_system.h>
#include "Ps4Cal.h"

// Settings
#define CTRL_MAC ""               // "" = use the ESP32's own Bluetooth address
#define TICK_MS 10
#define CENTER_MS 1500
#define RANGE_MS 8000
#define DETECT_COUNTS 80          // how far from centre counts as "pushed fully"
#define DETECT_HOLD_MS 500
#define DEADZONE_MIN 6
#define DEADZONE_MAX 20
#define CENTER_MAX_OFFSET 40      // reject a centre further off than this
#define CENTER_MAX_NOISE 12       // reject if the stick moved this much while capturing
#define MIN_TRAVEL 60             // each side must reach at least this far

enum Step
{
  WAIT_CONN,
  CENTER_READY,
  CENTER_CAP,
  RANGE_READY,
  RANGE_CAP,
  FWD_DETECT,
  RIGHT_DETECT,
  VERIFY
};

static const char *AXIS_NAME[4] = {"LX", "LY", "RX", "RY"};

static Ps4Calibration cal;
static Step step = WAIT_CONN;
static uint32_t stepT = 0, lastTick = 0, lastPrint = 0, holdT = 0;
static int holdSign = 0;
static float sum[4];
static int lo[4], hi[4];
static uint32_t cnt = 0;
static bool prevCross = false, prevOptions = false, prevShare = false;
static bool cmdNext = false, cmdSave = false, cmdRestart = false;

static void led(uint8_t r, uint8_t g, uint8_t b)
{
  if (!PS4.isConnected()) return;
  PS4.setLed(r, g, b);
  PS4.sendToController();
}

static void enter(Step s)
{
  step = s;
  stepT = millis();
  lastPrint = stepT;
  holdT = 0;
  holdSign = 0;

  switch (s)
  {
    case WAIT_CONN:
      Serial.println("\nWaiting for the PS4 controller. Press the PS button.");
      break;
    case CENTER_READY:
      led(0, 0, 255);
      Serial.println("\nSTEP 1/4 - CENTRE: release both sticks, then press X (or send n).");
      break;
    case CENTER_CAP:
      led(255, 80, 0);
      Serial.println("Capturing centre, keep your hands off the sticks...");
      break;
    case RANGE_READY:
      led(0, 0, 255);
      Serial.println("\nSTEP 2/4 - RANGE: press X (or send n), then roll BOTH sticks around their full edge for 8 s.");
      break;
    case RANGE_CAP:
      led(255, 80, 0);
      Serial.println("Go! Roll both sticks around the outer edge...");
      break;
    case FWD_DETECT:
      led(160, 0, 255);
      Serial.println("\nSTEP 3/4 - DIRECTION: push the LEFT stick fully FORWARD (away from you) and hold it.");
      break;
    case RIGHT_DETECT:
      led(160, 0, 255);
      Serial.println("\nSTEP 4/4 - DIRECTION: push the RIGHT stick fully to the RIGHT and hold it.");
      break;
    case VERIFY:
      led(0, 255, 0);
      Serial.println("\nLIVE CHECK: FORWARD should go + when you push the left stick forward, STEER + when you push the right stick right.");
      Serial.println("Release the sticks: both must read exactly 0.00. Options = save, Share = redo.");
      break;
  }
}

static void startCapture()
{
  for (int i = 0; i < 4; i++)
  {
    sum[i] = 0.0f;
    lo[i] = 999;
    hi[i] = -999;
  }
  cnt = 0;
}

static void sampleAll()
{
  for (int i = 0; i < 4; i++)
  {
    int v = ps4Raw(i);
    sum[i] += v;
    if (v < lo[i]) lo[i] = v;
    if (v > hi[i]) hi[i] = v;
  }
  cnt++;
}

static void finishCenter()
{
  bool bad = false;

  for (int i = 0; i < 4; i++)
  {
    int c = (int)lroundf(sum[i] / (float)cnt);
    int noise = max(hi[i] - c, c - lo[i]);

    cal.center[i] = c;
    cal.deadzone[i] = constrain(noise * 2 + 2, DEADZONE_MIN, DEADZONE_MAX);
    Serial.printf("  %s  centre %4d   noise +/-%d   deadzone %d\n", AXIS_NAME[i], c, noise, cal.deadzone[i]);

    if (abs(c) > CENTER_MAX_OFFSET || noise > CENTER_MAX_NOISE) bad = true;
  }

  if (bad)
  {
    Serial.println("A stick moved or was held off-centre. Try again.");
    enter(CENTER_READY);
    return;
  }
  enter(RANGE_READY);
}

static void finishRange()
{
  bool bad = false;

  for (int i = 0; i < 4; i++)
  {
    Serial.printf("  %s  min %4d   max %4d\n", AXIS_NAME[i], lo[i], hi[i]);
    if (hi[i] - cal.center[i] < MIN_TRAVEL || cal.center[i] - lo[i] < MIN_TRAVEL)
    {
      Serial.printf("  ^ %s did not reach full travel on both sides\n", AXIS_NAME[i]);
      bad = true;
    }
    cal.minRaw[i] = lo[i];
    cal.maxRaw[i] = hi[i];
  }

  if (bad)
  {
    Serial.println("Roll the sticks all the way around the edge. Try again.");
    enter(RANGE_READY);
    return;
  }
  enter(FWD_DETECT);
}

// True once the axis has been held fully to one side for DETECT_HOLD_MS; sign = side
static bool detectDirection(int axis, int &sign)
{
  int d = ps4Raw(axis) - cal.center[axis];
  int s = (d >= DETECT_COUNTS) ? 1 : (d <= -DETECT_COUNTS) ? -1 : 0;

  if (s == 0 || s != holdSign)
  {
    holdSign = s;
    holdT = millis();
    return false;
  }
  if (millis() - holdT >= DETECT_HOLD_MS)
  {
    sign = s;
    return true;
  }
  return false;
}

static void printLive()
{
  int ly = ps4Raw(P4AX_LY), rx = ps4Raw(P4AX_RX);
  float fwd = cal.fwdSign * ps4Norm(cal, P4AX_LY, ly);
  float str = cal.rightSign * ps4Norm(cal, P4AX_RX, rx);
  Serial.printf("FORWARD=%+5.2f  STEER=%+5.2f    (raw LY=%4d RX=%4d)\n", fwd, str, ly, rx);
}

static void doSave()
{
  if (ps4CalSave(cal))
  {
    Serial.println("Saved to flash. The drive firmware will load it at boot.");
    led(0, 255, 0);
  }
  else
  {
    Serial.println("Save failed: calibration is not valid. Press Share to redo.");
  }
}

static void readSerial()
{
  while (Serial.available())
  {
    char c = (char)tolower(Serial.read());
    if (c == 'n') cmdNext = true;
    else if (c == 's') cmdSave = true;
    else if (c == 'r') cmdRestart = true;
    else if (c == 'e')
    {
      ps4CalErase();
      Serial.println("Saved PS4 calibration erased.");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("\nPS4 stick calibration");

  ps4CalDefaults(cal);
  Ps4Calibration old;
  if (ps4CalLoad(old)) Serial.println("A saved calibration exists. Saving again will replace it.");

  if (strlen(CTRL_MAC))
  {
    PS4.begin(CTRL_MAC);
    Serial.printf("ESP32 Bluetooth address (set in code): %s\n", CTRL_MAC);
  }
  else
  {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    PS4.begin();
    Serial.printf("ESP32 Bluetooth address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  Serial.println("Pair the controller to this address (SixaxisPairTool) if it will not connect.");

  enter(WAIT_CONN);
}

void loop()
{
  readSerial();

  uint32_t now = millis();
  if (now - lastTick < TICK_MS) return;
  lastTick = now;

  bool conn = PS4.isConnected();
  bool cross = conn && PS4.Cross();
  bool options = conn && PS4.Options();
  bool share = conn && PS4.Share();

  bool go = (cross && !prevCross) || cmdNext;
  bool save = (options && !prevOptions) || cmdSave;
  bool redo = (share && !prevShare) || cmdRestart;
  prevCross = cross;
  prevOptions = options;
  prevShare = share;
  cmdNext = cmdSave = cmdRestart = false;

  if (!conn)
  {
    if (step != WAIT_CONN)
    {
      Serial.println("Controller lost.");
      enter(WAIT_CONN);
    }
    return;
  }

  if (step == WAIT_CONN)
  {
    Serial.println("Controller connected.");
    enter(CENTER_READY);
    return;
  }

  if (redo)
  {
    ps4CalDefaults(cal);
    enter(CENTER_READY);
    return;
  }

  switch (step)
  {
    case CENTER_READY:
      if (go)
      {
        startCapture();
        enter(CENTER_CAP);
      }
      break;

    case CENTER_CAP:
      sampleAll();
      if (now - stepT >= CENTER_MS) finishCenter();
      break;

    case RANGE_READY:
      if (go)
      {
        for (int i = 0; i < 4; i++) lo[i] = hi[i] = cal.center[i];
        cnt = 0;
        enter(RANGE_CAP);
      }
      break;

    case RANGE_CAP:
      sampleAll();
      if (now - stepT >= RANGE_MS)
      {
        finishRange();
      }
      else if (now - lastPrint >= 1000)
      {
        lastPrint = now;
        Serial.printf("  %d s left\n", (int)((RANGE_MS - (now - stepT) + 999) / 1000));
      }
      break;

    case FWD_DETECT:
    {
      int s;
      if (detectDirection(P4AX_LY, s))
      {
        cal.fwdSign = (int8_t)s;
        Serial.printf("  Forward = LStickY goes %s\n", s > 0 ? "positive" : "negative");
        enter(RIGHT_DETECT);
      }
      break;
    }

    case RIGHT_DETECT:
    {
      int s;
      if (detectDirection(P4AX_RX, s))
      {
        cal.rightSign = (int8_t)s;
        Serial.printf("  Right = RStickX goes %s\n", s > 0 ? "positive" : "negative");
        enter(VERIFY);
      }
      break;
    }

    case VERIFY:
      if (save) doSave();
      if (now - lastPrint >= 100)
      {
        lastPrint = now;
        printLive();
      }
      break;

    default:
      break;
  }
}
