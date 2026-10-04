/*
ENCODER TEST

Purpose:
Measure the encoder pulses per wheel revolution.

Pins (you can use preferred pins):
LEFT  / Motor A -> GPIO 18
RIGHT / Motor B -> GPIO 19

How to use:
1. Upload the sketch and open Serial Monitor at 115200 baud.
2. Keep the motors OFF and rotate the wheels by hand.
3. Type A to test the left wheel or B to test the right wheel. 
4. Rotate the selected wheel exactly 10 complete revolutions. The program does not count the 10 revolutions automatically. You must count
the physical wheel revolutions yourself.
5. Type DONE and press Enter to display the result.
6. Repeat each test 3 times for consistency.

Commands:
A     = Start LEFT test
B     = Start RIGHT test
DONE  = Finish active test and calculate counts/revolution
R     = Reset both counters
S     = Show current raw encoder counts
X     = Cancel active test

Note:
Counts per revolution = Total pulses / 10
*/

#include <Arduino.h>

// Encoder pins
const uint8_t ENCODER_A_PIN = 18;   // Motor A - LEFT
const uint8_t ENCODER_B_PIN = 19;   // Motor B - RIGHT

// Test settings
const int TEST_REVOLUTIONS = 10;
const unsigned long PRINT_INTERVAL = 500;


volatile uint32_t encoderACount = 0;
volatile uint32_t encoderBCount = 0;

// Test state
enum TestWheel {
  TEST_NONE,
  TEST_LEFT,
  TEST_RIGHT
};

TestWheel activeTest = TEST_NONE;

uint32_t startCount = 0;
unsigned long lastPrint = 0;

// Interrupt service routines
void IRAM_ATTR encoderA_ISR() {
  encoderACount++;
}

void IRAM_ATTR encoderB_ISR() {
  encoderBCount++;
}

uint32_t getACount() {
  noInterrupts();
  uint32_t value = encoderACount;
  interrupts();
  return value;
}

uint32_t getBCount() {
  noInterrupts();
  uint32_t value = encoderBCount;
  interrupts();
  return value;
}


void resetCounters() {
  noInterrupts();
  encoderACount = 0;
  encoderBCount = 0;
  interrupts();
}

void startTest(TestWheel wheel) {

  activeTest = wheel;

  uint32_t currentCount;

  if (wheel == TEST_LEFT) {
    currentCount = getACount();
    startCount = currentCount;

    Serial.println();
    Serial.println(F(" "));
    Serial.println(F("LEFT / MOTOR A TEST"));
    Serial.println(F("Encoder: GPIO 18"));
    Serial.println(F("Target: 10 complete wheel revolutions"));
    Serial.println(F("Rotate the marked wheel manually."));
    Serial.println(F("After exactly 10 revolutions, type DONE."));
    Serial.println(F(" "));
  }

  else if (wheel == TEST_RIGHT) {
    currentCount = getBCount();
    startCount = currentCount;

    Serial.println();
    Serial.println(F(" "));
    Serial.println(F("RIGHT / MOTOR B TEST"));
    Serial.println(F("Encoder: GPIO 19"));
    Serial.println(F("Target: 10 complete wheel revolutions"));
    Serial.println(F("Rotate the marked wheel manually."));
    Serial.println(F("After exactly 10 revolutions, type DONE."));
    Serial.println(F(" "));
  }

  lastPrint = millis();
}

// Finish test
void finishTest() {

  uint32_t endCount;
  uint32_t pulses;

  if (activeTest == TEST_LEFT) {

    endCount = getACount();
    pulses = endCount - startCount;

    float countsPerRev = (float)pulses / TEST_REVOLUTIONS;

    Serial.println();
    Serial.println(F(" "));
    Serial.println(F("LEFT - MOTOR A RESULT"));
    Serial.print(F("Total pulses: "));
    Serial.println(pulses);
    Serial.print(F("Revolutions: "));
    Serial.println(TEST_REVOLUTIONS);
    Serial.print(F("Counts per revolution: "));
    Serial.println(countsPerRev, 4);
    Serial.println(F(" "));
  }

  else if (activeTest == TEST_RIGHT) {

    endCount = getBCount();
    pulses = endCount - startCount;
    float countsPerRev = (float)pulses / TEST_REVOLUTIONS;

    Serial.println();
    Serial.println(F(" "));
    Serial.println(F("RIGHT - MOTOR B RESULT"));
    Serial.print(F("Total pulses: "));
    Serial.println(pulses);
    Serial.print(F("Revolutions: "));
    Serial.println(TEST_REVOLUTIONS);
    Serial.print(F("Counts per revolution: "));
    Serial.println(countsPerRev, 4);
    Serial.println(F(" "));
  }

  activeTest = TEST_NONE;

  Serial.println();
  Serial.println(F("Next command:"));
  Serial.println(F("A = test LEFT"));
  Serial.println(F("B = test RIGHT"));
  Serial.println(F("R = reset counters"));
  Serial.println(F("S = show counters"));
  Serial.println();
}


void showCounters() {
  Serial.println();
  Serial.println(F("CURRENT RAW COUNTS"));
  Serial.print(F("Motor A / LEFT  (GPIO18): "));
  Serial.println(getACount());
  Serial.print(F("Motor B / RIGHT (GPIO19): "));
  Serial.println(getBCount());
  Serial.println();
}


void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "A") {
    activeTest = TEST_NONE;
    startTest(TEST_LEFT);
  }

  else if (command == "B") {
    activeTest = TEST_NONE;
    startTest(TEST_RIGHT);
  }

  else if (command == "DONE") {
    if (activeTest == TEST_NONE) {
      Serial.println(F("No test is currently running."));
    } else {
      finishTest();
    }
  }

  else if (command == "R") {
    activeTest = TEST_NONE;

    resetCounters();
    Serial.println();
    Serial.println(F("Both encoder counters reset to 0."));
    Serial.println();
  }

  else if (command == "S") {
    showCounters();
  }

  else if (command == "X") {
    activeTest = TEST_NONE;

    Serial.println();
    Serial.println(F("Active test cancelled."));
    Serial.println();
  }

  else {
    Serial.println();
    Serial.println(F("Unknown command."));
    Serial.println(F("Use A, B, DONE, R, S, or X."));
    Serial.println();
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);

  // LM393 encoder outputs
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A_PIN),
    encoderA_ISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_B_PIN),
    encoderB_ISR,
    RISING
  );

  resetCounters();
  Serial.println();
  Serial.println(F("       ENCODER PPR TEST PROGRAM"));
  Serial.println(F(" "));
  Serial.println(F("LEFT  - Motor A - GPIO 18"));
  Serial.println(F("RIGHT - Motor B - GPIO 19"));
  Serial.println();
  Serial.println(F("Recommended procedure:"));
  Serial.println(F("1. Put a visible mark on the wheel."));
  Serial.println(F("2. Keep the motor completely OFF."));
  Serial.println(F("3. Type A to test the left wheel."));
  Serial.println(F("4. Rotate it exactly 10 full revolutions."));
  Serial.println(F("5. Type DONE."));
  Serial.println(F("6. Repeat 3 times."));
  Serial.println(F("7. Do the same for B."));
  Serial.println();
  Serial.println(F("Commands: A, B, DONE, R, S, X"));
  Serial.println();
}


void loop() {

  handleSerial();

  // Live pulse display while a test is active
  if (activeTest != TEST_NONE && millis() - lastPrint >= PRINT_INTERVAL) { 
      lastPrint = millis();
      uint32_t count;

      if (activeTest == TEST_LEFT) {
        count = getACount();

        Serial.print(F("LEFT pulses: "));
        Serial.println(count - startCount);
      }

    else {
      count = getBCount();

      Serial.print(F("RIGHT pulses: "));
      Serial.println(count - startCount);
    }
  }
}
