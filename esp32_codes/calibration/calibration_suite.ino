/*
  ==========================================================================
  ESP32 dual-motor OPEN-LOOP calibration rig  (pairs with calibrate_host.py)
  ==========================================================================
  This firmware has NO PID, NO feedforward, NO ramping. It is a dumb,
  well-instrumented actuator: the host script commands raw signed PWM and
  receives a timestamped stream of encoder counts. All test sequencing and
  curve fitting happens on the PC, so changing a test never needs a reflash.

  Convention (physical, un-inverted for BOTH motors):
      +PWM -> LPWM side -> clockwise -> encoder counts INCREASE
      -PWM -> RPWM side -> anticlockwise -> encoder counts DECREASE
  Motor index 0 = A (RPWM=32, LPWM=33, ENC=16/17)
  Motor index 1 = B (RPWM=25, LPWM=26, ENC=18/19)

  SAFETY (all on by default)
    - Watchdog: motors stop if no command/keepalive for 1000 ms.
    - Reversal interlock: a command opposite to the current rotation
      direction is held at 0 (BTS7960 short-brake) until measured speed
      falls below INTERLOCK_DEGS. Prevents plugging the drivers during tests.
    - PWM clamped to +-PWM_LIMIT.

  SERIAL (460800 baud, newline terminated)
      M <0|1> <pwm>     set one motor (signed)
      B <pwmA> <pwmB>   set both motors in one command
      S                 stop both (PWM 0 = short-brake)
      Z                 zero both encoder counts
      R <hz>            stream rate, 10..500 (default 200)
      T <0|1>           stream off/on
      I <0|1>           reversal interlock off/on (default 1)
      K                 keepalive (feeds watchdog)
      P                 print one data line
      V                 version/handshake
  STREAM LINE:  D,<t_us>,<countA>,<countB>,<appliedPwmA>,<appliedPwmB>

  Uses the classic Arduino-ESP32 core v2.x LEDC API (same as your other
  sketches). On core v3.x replace ledcSetup/ledcAttachPin/ledcWrite(ch,..)
  with ledcAttach(pin,freq,res)/ledcWrite(pin,duty).
  ==========================================================================
*/

#include <Arduino.h>

// ---------------- pins ----------------
#define MOTOR_A_RPWM 32
#define MOTOR_A_LPWM 33
#define ENC_A_C1     16
#define ENC_A_C2     17

#define MOTOR_B_RPWM 25
#define MOTOR_B_LPWM 26
#define ENC_B_C1     18
#define ENC_B_C2     19

// ---------------- constants ----------------
const double   CPR            = 5070.0;      // confirmed output-shaft CPR (both motors)
const int      PWM_FREQ_HZ    = 20000;
const int      PWM_RES_BITS   = 8;
const int      PWM_LIMIT      = 230;         // raise only if you intend to test above 230
const unsigned long WDOG_MS   = 1000;
const double   INTERLOCK_DEGS = 30.0;        // deg/s; opposite-direction drive blocked above this
const uint32_t SPD_WINDOW_US  = 40000;       // speed estimate for the interlock only

// ---------------- state ----------------
struct Motor {
  int pinR, pinL, chR, chL;
  volatile long count;
  volatile uint8_t last;
  int cmd;          // requested signed PWM
  int applied;      // signed PWM actually on the pins
  long spdCount;
  uint32_t spdT;
  double spd;       // deg/s, for the interlock
};

Motor mot[2];

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

bool     streaming    = true;
bool     interlockOn  = true;
uint32_t streamPeriodUs = 5000;   // 200 Hz
uint32_t lastStreamUs = 0;
unsigned long lastCmdMs = 0;
bool     ilPrinted[2] = {false, false};
bool     wdPrinted    = false;

// ---------------- encoder ISRs ----------------
void IRAM_ATTR isrA() {
  uint8_t s = (digitalRead(ENC_A_C1) << 1) | digitalRead(ENC_A_C2);
  mot[0].count += QUAD_TABLE[(mot[0].last << 2) | s];
  mot[0].last = s;
}

void IRAM_ATTR isrB() {
  uint8_t s = (digitalRead(ENC_B_C1) << 1) | digitalRead(ENC_B_C2);
  mot[1].count += QUAD_TABLE[(mot[1].last << 2) | s];
  mot[1].last = s;
}

// ---------------- output ----------------
void drive(Motor &m, int pwm) {
  pwm = constrain(pwm, -PWM_LIMIT, PWM_LIMIT);
  if (pwm > 0) {
    ledcWrite(m.chR, 0);
    ledcWrite(m.chL, pwm);
  } else if (pwm < 0) {
    ledcWrite(m.chL, 0);
    ledcWrite(m.chR, -pwm);
  } else {
    ledcWrite(m.chL, 0);
    ledcWrite(m.chR, 0);
  }
  m.applied = pwm;
}

void updateSpeeds() {
  uint32_t now = micros();
  for (int i = 0; i < 2; i++) {
    uint32_t dt = now - mot[i].spdT;
    if (dt >= SPD_WINDOW_US) {
      noInterrupts();
      long c = mot[i].count;
      interrupts();
      mot[i].spd = (double)(c - mot[i].spdCount) * 360.0 / CPR / (dt * 1e-6);
      mot[i].spdCount = c;
      mot[i].spdT = now;
    }
  }
}

void applyCommands() {
  for (int i = 0; i < 2; i++) {
    int req = mot[i].cmd;
    bool blocked = false;
    if (interlockOn && req != 0 && fabs(mot[i].spd) > INTERLOCK_DEGS &&
        ((req > 0) != (mot[i].spd > 0))) {
      req = 0;      // brake first, never plug
      blocked = true;
    }
    if (blocked && !ilPrinted[i]) {
      Serial.print(F("INTERLOCK motor "));
      Serial.println(i);
      ilPrinted[i] = true;
    }
    if (!blocked) ilPrinted[i] = false;
    if (req != mot[i].applied) drive(mot[i], req);
  }
}

void stopAll() {
  mot[0].cmd = 0;
  mot[1].cmd = 0;
}

// ---------------- streaming ----------------
void printDataLine() {
  noInterrupts();
  long a = mot[0].count;
  long b = mot[1].count;
  uint32_t t = micros();
  interrupts();
  char buf[72];
  int n = snprintf(buf, sizeof(buf), "D,%lu,%ld,%ld,%d,%d\n",
                   (unsigned long)t, a, b, mot[0].applied, mot[1].applied);
  Serial.write((const uint8_t *)buf, n);
}

// ---------------- command parser ----------------
void handleLine(char *s) {
  int i, p, pa, pb, v;
  lastCmdMs = millis();
  wdPrinted = false;

  if (s[0] == 'M' && sscanf(s, "M %d %d", &i, &p) == 2 && (i == 0 || i == 1)) {
    mot[i].cmd = constrain(p, -PWM_LIMIT, PWM_LIMIT);
  } else if (s[0] == 'B' && sscanf(s, "B %d %d", &pa, &pb) == 2) {
    mot[0].cmd = constrain(pa, -PWM_LIMIT, PWM_LIMIT);
    mot[1].cmd = constrain(pb, -PWM_LIMIT, PWM_LIMIT);
  } else if (s[0] == 'S') {
    stopAll();
  } else if (s[0] == 'Z') {
    noInterrupts();
    mot[0].count = 0;
    mot[1].count = 0;
    interrupts();
    mot[0].spdCount = 0;
    mot[1].spdCount = 0;
  } else if (s[0] == 'R' && sscanf(s, "R %d", &v) == 1) {
    v = constrain(v, 10, 500);
    streamPeriodUs = 1000000UL / v;
  } else if (s[0] == 'T' && sscanf(s, "T %d", &v) == 1) {
    streaming = (v != 0);
  } else if (s[0] == 'I' && sscanf(s, "I %d", &v) == 1) {
    interlockOn = (v != 0);
    Serial.print(F("INTERLOCK "));
    Serial.println(interlockOn ? F("ON") : F("OFF"));
  } else if (s[0] == 'K') {
    // keepalive: lastCmdMs already refreshed
  } else if (s[0] == 'P') {
    printDataLine();
  } else if (s[0] == 'V') {
    Serial.print(F("CAL,v1,CPR="));
    Serial.print(CPR, 0);
    Serial.print(F(",limit="));
    Serial.println(PWM_LIMIT);
  } else {
    Serial.println(F("ERR unknown command"));
  }
}

void pollSerial() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        buf[n] = 0;
        handleLine(buf);
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;    // overlong line: discard
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(460800);
  delay(200);

  mot[0] = {MOTOR_A_RPWM, MOTOR_A_LPWM, 0, 1, 0, 0, 0, 0, 0, 0, 0.0};
  mot[1] = {MOTOR_B_RPWM, MOTOR_B_LPWM, 2, 3, 0, 0, 0, 0, 0, 0, 0.0};

  pinMode(ENC_A_C1, INPUT_PULLUP);
  pinMode(ENC_A_C2, INPUT_PULLUP);
  pinMode(ENC_B_C1, INPUT_PULLUP);
  pinMode(ENC_B_C2, INPUT_PULLUP);
  mot[0].last = (digitalRead(ENC_A_C1) << 1) | digitalRead(ENC_A_C2);
  mot[1].last = (digitalRead(ENC_B_C1) << 1) | digitalRead(ENC_B_C2);

  for (int i = 0; i < 2; i++) {
    ledcSetup(mot[i].chR, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(mot[i].chL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(mot[i].pinR, mot[i].chR);
    ledcAttachPin(mot[i].pinL, mot[i].chL);
    ledcWrite(mot[i].chR, 0);
    ledcWrite(mot[i].chL, 0);
    mot[i].spdT = micros();
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A_C1), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_C2), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_C1), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_C2), isrB, CHANGE);

  lastCmdMs = millis();
  Serial.println(F("CAL,v1,ready"));
}

void loop() {
  pollSerial();
  updateSpeeds();

  if ((mot[0].cmd != 0 || mot[1].cmd != 0) && millis() - lastCmdMs > WDOG_MS) {
    stopAll();
    if (!wdPrinted) {
      Serial.println(F("WATCHDOG: no command, motors stopped"));
      wdPrinted = true;
    }
  }

  applyCommands();

  uint32_t now = micros();
  if (streaming && (uint32_t)(now - lastStreamUs) >= streamPeriodUs) {
    lastStreamUs = now;
    printDataLine();
  }
}
