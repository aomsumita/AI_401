/*
 * esp32_stepper.ino  —  ESP32 คุม stepper 1 แกน ผ่าน TB6600 (PUL/DIR)
 *
 * ค่าที่ทดสอบแล้ว:  800 pulse/rev, 1 รอบ = 8 cm  ->  100 steps = 1 cm  (1 step = 0.1 mm)
 * ระยะทำงาน 60 cm  ->  0 ถึง 6000 steps
 *
 * การต่อสาย TB6600:
 *   PUL+ -> GPIO 16        DIR+ -> GPIO 17
 *   PUL-, DIR-, ENA+, ENA- -> GND ของ ESP32
 *   limit switch (NO): ขาหนึ่ง -> GPIO 4, อีกขา -> GND   (ติดไว้ที่ปลายราง "ทิศ r")
 *
 * Library: AccelStepper (Mike McCauley) — ติดตั้งจาก Library Manager
 *
 * คำสั่ง (Serial 115200, ตั้ง Serial Monitor เป็น Newline)   หน่วยเป็น steps (100 steps = 1 cm)
 *   Pi -> ESP32          ESP32 -> Pi
 *   PING                 PONG
 *   HOME                 OK ... DONE x      วิ่งหา limit switch แล้วตั้งเป็น 0
 *   MOVE x               OK ... DONE x      ไปตำแหน่ง x  (เช่น MOVE 1500 = ไปที่ 15 cm)
 *   MOVR dx              OK ... DONE x      ขยับเพิ่ม dx (เช่น MOVR -50 = ถอย 0.5 cm) ใช้ตอน PID
 *   POS?                 POS x
 *   STOP                 OK                 หยุด (ใช้ได้ทั้งตอน MOVE และ HOME)
 *   SPEED v a            OK                 ตั้งความเร็ว (steps/s) และอัตราเร่ง (steps/s^2)
 *   (ระหว่างวิ่ง ESP32 ส่ง "P x" ทุก 0.1 s เพื่อให้ Pi แสดงตำแหน่งสดบนหน้าจอ)
 *   ผิดพลาด: ERR BUSY | ERR NOT_HOMED | ERR LIMIT | ERR HOME_TIMEOUT | ERR HOME_STOPPED | ERR UNKNOWN
 */
#include <AccelStepper.h>

// ---------- Pins ----------
#define PUL_PIN    16
#define DIR_PIN    17
#define LIMIT_PIN  4      // แก้ให้ตรงกับขาที่ต่อ limit switch จริง

// ถ้าสั่ง HOME แล้วมอเตอร์วิ่ง "หนี" switch ให้เปลี่ยนเป็น true
const bool DIR_INVERT = true;

// ---------- ค่าเครื่อง (จากการทดสอบ) ----------
const float PULSE_PER_REV = 800;                          // DIP switch TB6600
const float CM_PER_REV    = 8.0;                          // วัดจริง
const float STEPS_PER_CM  = PULSE_PER_REV / CM_PER_REV;   // = 100
const float TRAVEL_CM     = 60.0;                         // ระยะทำงาน
const long  X_MAX = (long)(TRAVEL_CM * STEPS_PER_CM);     // = 4000 steps

float maxSpeed = 625;      // steps/s = 6.25 cm/s  (เท่ากับ pulseDelay 800 us ที่ทดสอบผ่าน)
float accel    = 1250;     // steps/s^2  (ถึงความเร็วสูงสุดใน 0.5 s)

const float HOME_SPEED   = 300;     // 3 cm/s  วิ่งหา switch ช้ากว่าปกติ
const long  HOME_BACKOFF = 50;      // ถอยออกจาก switch 0.5 cm แล้วตั้งเป็น 0
const unsigned long HOME_TIMEOUT_MS = 30000;   // 40 cm ที่ 3 cm/s ใช้ ~13 s
const int   PULSE_US = 10;          // ความกว้างพัลส์ (TB6600 ต้อง >= 2.5 us)

AccelStepper motor(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

bool moving = false;
bool homed  = false;
String buf;

enum HomeResult { HOME_OK, HOME_TIMEOUT, HOME_STOPPED };

long clampL(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }
bool limitHit() { return digitalRead(LIMIT_PIN) == LOW; }

void reportDone() { Serial.printf("DONE %ld\n", motor.currentPosition()); }

// อ่าน 1 บรรทัดแบบไม่บล็อก
bool readLine(String &out) {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length()) { out = buf; buf = ""; return true; }
    } else if (buf.length() < 64) {
      buf += ch;
    }
  }
  return false;
}

// ระหว่าง HOME: รับเฉพาะ STOP คำสั่งอื่นตอบ BUSY
bool stopRequested() {
  String l;
  if (!readLine(l)) return false;
  l.trim();
  l.toUpperCase();
  if (l == "STOP") return true;
  Serial.println("ERR BUSY");
  return false;
}

HomeResult homeAxis() {
  // ถ้ากด switch ค้างอยู่แล้ว ให้ถอยออกก่อน
  if (limitHit()) {
    motor.move(HOME_BACKOFF);
    motor.runToPosition();
  }
  motor.setSpeed(-HOME_SPEED);                 // วิ่งเข้าหา switch (ทิศลบ = ทิศ r)
  unsigned long t0 = millis();
  while (!limitHit()) {
    motor.runSpeed();
    if (stopRequested()) return HOME_STOPPED;
    if (millis() - t0 > HOME_TIMEOUT_MS) return HOME_TIMEOUT;
  }
  motor.setCurrentPosition(0);
  motor.move(HOME_BACKOFF);                    // ถอยออกจาก switch
  motor.runToPosition();
  motor.setCurrentPosition(0);                 // จุดนี้คือ 0 cm
  return HOME_OK;
}

void startMove(long x) {
  long tx = clampL(x, 0, X_MAX);
  if (tx != x) Serial.println("WARN SOFT_LIMIT");
  motor.moveTo(tx);
  moving = true;
}

void handle(String line) {
  line.trim();
  if (!line.length()) return;
  char cmd[8] = {0};
  long a = 0, b = 0;
  int n = sscanf(line.c_str(), "%7s %ld %ld", cmd, &a, &b);
  String c(cmd);
  c.toUpperCase();

  if (c == "PING") { Serial.println("PONG"); return; }
  if (c == "POS?") { Serial.printf("POS %ld\n", motor.currentPosition()); return; }
  if (c == "STOP") { motor.stop(); Serial.println("OK"); return; }   // DONE ตามมาเมื่อหยุดสนิท
  if (moving)      { Serial.println("ERR BUSY"); return; }

  if (c == "HOME") {
    Serial.println("OK");
    HomeResult r = homeAxis();
    if (r != HOME_OK) {
      homed = false;
      Serial.println(r == HOME_STOPPED ? "ERR HOME_STOPPED" : "ERR HOME_TIMEOUT");
      return;
    }
    homed = true;
    reportDone();
  } else if (c == "MOVE" && n >= 2) {
    if (!homed) { Serial.println("ERR NOT_HOMED"); return; }
    Serial.println("OK");
    startMove(a);
  } else if (c == "MOVR" && n >= 2) {
    if (!homed) { Serial.println("ERR NOT_HOMED"); return; }
    Serial.println("OK");
    startMove(motor.currentPosition() + a);
  } else if (c == "SPEED" && n == 3) {
    maxSpeed = a; accel = b;
    motor.setMaxSpeed(maxSpeed);
    motor.setAcceleration(accel);
    Serial.println("OK");
  } else {
    Serial.println("ERR UNKNOWN");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  motor.setPinsInverted(DIR_INVERT, false, false);
  motor.setMinPulseWidth(PULSE_US);
  motor.setMaxSpeed(maxSpeed);
  motor.setAcceleration(accel);

  Serial.printf("READY  (1 cm = %.0f steps, travel 0-%ld steps)\n", STEPS_PER_CM, X_MAX);
}

void loop() {
  String line;
  if (readLine(line)) handle(line);

  if (moving) {
    // ความปลอดภัย: วิ่งทิศลบแล้วชน switch -> หยุดทันที ต้อง HOME ใหม่
    if (limitHit() && motor.speed() < 0) {
      motor.setCurrentPosition(motor.currentPosition());
      moving = false;
      homed = false;
      Serial.println("ERR LIMIT");
      return;
    }
    motor.run();

    // รายงานตำแหน่งระหว่างวิ่งทุก 100 ms (ให้ Pi แสดงตำแหน่งสด)
    static unsigned long lastReport = 0;
    if (millis() - lastReport >= 100) {
      lastReport = millis();
      Serial.printf("P %ld\n", motor.currentPosition());
    }

    if (motor.distanceToGo() == 0) {
      moving = false;
      reportDone();
    }
  }
}