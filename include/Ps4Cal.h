#pragma once
/*
 * Ps4Cal.h - Shared PS4 stick calibration helpers

 * WHAT IT DOES:
    Defines the stick calibration record (centre, travel, deadzone, direction),
    flash load/save, and the conversion from raw stick counts to a clean
    -1..+1 value. Shared by the PS4 calibration tool and the drive firmware.

 * HOW TO USE:
    Copy into the include/ folder of the balancing project. Run the PS4
    calibration tool once, then call ps4CalLoad() in setup() of the drive
    firmware and ps4Norm() / ps4Raw() in loop().

 * NOTE:
    Raw stick values are the PS4Controller library's int8 counts (-128..127).
    Direction (which sign means "forward" / "right") is measured by the tool and
    stored in the record, so the library's up/right polarity does not matter.
    Calibration lives in NVS, namespace "ps4cal", key "cal".
*/

#include <Arduino.h>
#include <Preferences.h>
#include <PS4Controller.h>
#include <math.h>
#include <string.h>

#define P4CAL_MAGIC    0x50533443UL
#define P4CAL_VERSION  1

enum Ps4Axis { P4AX_LX = 0, P4AX_LY = 1, P4AX_RX = 2, P4AX_RY = 3 };

struct Ps4Calibration
{
  uint32_t magic;
  uint32_t version;
  int16_t center[4];     // raw counts at rest (LX, LY, RX, RY)
  int16_t minRaw[4];     // raw counts, full travel, negative side
  int16_t maxRaw[4];     // raw counts, full travel, positive side
  int16_t deadzone[4];   // counts around centre that read as exactly 0
  int8_t  fwdSign;       // +1 if pushing the left stick forward raises LStickY()
  int8_t  rightSign;     // +1 if pushing the right stick right raises RStickX()
  int8_t  reserved[2];
};

inline void ps4CalDefaults(Ps4Calibration &c)
{
  memset(&c, 0, sizeof(c));
  c.magic = P4CAL_MAGIC;
  c.version = P4CAL_VERSION;
  for (int i = 0; i < 4; i++)
  {
    c.center[i] = 0;
    c.minRaw[i] = -127;
    c.maxRaw[i] = 127;
    c.deadzone[i] = 8;
  }
  c.fwdSign = 1;
  c.rightSign = 1;
}

inline bool ps4CalValid(const Ps4Calibration &c)
{
  if (c.magic != P4CAL_MAGIC || c.version != P4CAL_VERSION) return false;

  for (int i = 0; i < 4; i++)
  {
    if (c.maxRaw[i] - c.center[i] < 60) return false;
    if (c.center[i] - c.minRaw[i] < 60) return false;
    if (c.deadzone[i] < 0 || c.deadzone[i] > 30) return false;
  }
  if (c.fwdSign != 1 && c.fwdSign != -1) return false;
  if (c.rightSign != 1 && c.rightSign != -1) return false;
  return true;
}

inline bool ps4CalLoad(Ps4Calibration &c)
{
  Preferences p;
  if (!p.begin("ps4cal", false)) return false;

  bool ok = false;
  if (p.getBytesLength("cal") == sizeof(Ps4Calibration))
  {
    Ps4Calibration tmp;
    p.getBytes("cal", &tmp, sizeof(tmp));
    if (ps4CalValid(tmp))
    {
      c = tmp;
      ok = true;
    }
  }
  p.end();
  return ok;
}

inline bool ps4CalSave(Ps4Calibration &c)
{
  c.magic = P4CAL_MAGIC;
  c.version = P4CAL_VERSION;
  if (!ps4CalValid(c)) return false;

  Preferences p;
  if (!p.begin("ps4cal", false)) return false;
  size_t n = p.putBytes("cal", &c, sizeof(c));
  p.end();
  return n == sizeof(c);
}

inline void ps4CalErase()
{
  Preferences p;
  if (p.begin("ps4cal", false))
  {
    p.remove("cal");
    p.end();
  }
}

// Raw stick counts straight from the library
inline int ps4Raw(int axis)
{
  switch (axis)
  {
    case P4AX_LX: return PS4.LStickX();
    case P4AX_LY: return PS4.LStickY();
    case P4AX_RX: return PS4.RStickX();
    default: return PS4.RStickY();
  }
}

// Raw counts -> -1..+1 (same polarity as the raw axis). Exactly 0 inside the
// deadzone, and the deadzone edge maps to 0 so there is no jump when leaving it.
inline float ps4Norm(const Ps4Calibration &c, int axis, int raw)
{
  float d = (float)(raw - c.center[axis]);
  float a = fabsf(d);
  float dz = (float)c.deadzone[axis];
  if (a <= dz) return 0.0f;

  float span = (d > 0.0f) ? (float)(c.maxRaw[axis] - c.center[axis]) : (float)(c.center[axis] - c.minRaw[axis]);
  float v = (a - dz) / (span - dz);
  if (v > 1.0f) v = 1.0f;
  return (d > 0.0f) ? v : -v;
}

// Softens the centre of the stick: 0 = linear, 1 = fully cubic
inline float ps4Expo(float v, float expo)
{
  return (1.0f - expo) * v + expo * v * v * v;
}
