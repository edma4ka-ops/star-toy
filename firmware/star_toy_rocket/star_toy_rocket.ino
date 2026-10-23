// ============================================================
//  «Звёздный курс» — бортовая прошивка ракеты (одна плата).
//
//  Что делает:
//    1) принимает команды оператора по RC (PPM или SBUS);
//    2) стабилизирует: гасит раскачку, держит заданный пилотом угол,
//       по тумблеру фиксирует текущее положение и удерживает его;
//    3) обнаруживает старт по перегрузке;
//    4) раскрывает парашюты: дрог в апогее, основной купол на высоте;
//    5) пишет лог в Serial; при потере сигнала — рули в нейтраль.
//
//  Чего НЕ делает (сознательно):
//    - не держит абсолютный курс по компасу/солнцу/звёздам;
//    - не летит по точкам маршрута.
//  Направление задаёт только человек: положением пусковой и стиками.
//
//  Расчёты, из которых взяты коэффициенты и площади рулей, — в star-toy/sim.
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include "config.h"

// ============================================================
//  1. RC
// ============================================================
volatile uint16_t ppmCh[9] = {0};
volatile uint8_t ppmIdx = 0;
volatile uint32_t ppmLastEdge = 0;
volatile bool ppmFresh = false;
uint16_t ch[9] = {0};
uint32_t lastFrameMs = 0;

void IRAM_ATTR ppmISR() {
  uint32_t nowUs = micros();
  uint32_t width = nowUs - ppmLastEdge;
  ppmLastEdge = nowUs;
  if (width > 3000) { ppmIdx = 0; ppmFresh = true; return; }
  if (ppmIdx < 9) ppmCh[ppmIdx++] = (uint16_t)width;
}

void rcInit() {
#if RC_PROTOCOL == 1
  pinMode(PIN_PPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PPM), ppmISR, RISING);
#else
  Serial2.begin(100000, SERIAL_8E2, PIN_SBUS_RX, -1, true);
#endif
}

bool rcRead() {
#if RC_PROTOCOL == 1
  bool fresh = false;
  noInterrupts();
  for (int i = 1; i <= 8; i++) ch[i] = ppmCh[i - 1];
  if (ppmFresh) { ppmFresh = false; fresh = true; }
  interrupts();
  if (!fresh) return false;
  for (int i = 1; i <= 8; i++) if (ch[i] < 800 || ch[i] > 2200) ch[i] = 1500;
  lastFrameMs = millis();
  return true;
#else
  static uint8_t buf[24];
  while (Serial2.available() >= 25) {
    if (Serial2.read() != 0x0F) continue;
    for (int i = 0; i < 24; i++) buf[i] = Serial2.read();
    if (!(buf[23] & 0x04) && !(buf[23] & 0x14) && !(buf[23] & 0x24)) { }
    for (int i = 0; i < 8; i++)
      ch[i + 1] = 1000 + ((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8)) * 1000 / 2047;
    lastFrameMs = millis();
    return true;
  }
  return false;
#endif
}

bool rcAlive() { return (millis() - lastFrameMs) < RC_FAILSAFE_MS; }

float stick(int channel) {
  int v = (int)ch[channel];
  if (abs(v - 1500) < RC_DEADBAND_US) return 0.0f;
  float x = (v - 1500) / 500.0f;
  return x > 1 ? 1 : (x < -1 ? -1 : x);
}

// ============================================================
//  2. IMU
// ============================================================
float gyro[3], acc[3];
float angX = 0, angY = 0, headingEst = 0;
uint32_t lastImuMs = 0;

void imuInit() {
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08);  // +-500 °/с
  Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10);  // +-8 g
  Wire.endTransmission();
}

void imuRead() {
  Wire.beginTransmission(0x68); Wire.write(0x3B);
  if (Wire.endTransmission(false)) return;
  if (Wire.requestFrom(0x68, 14) != 14) return;
  int16_t a[3], g[3];
  for (int i = 0; i < 3; i++) a[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read();
  for (int i = 0; i < 3; i++) g[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  for (int i = 0; i < 3; i++) {
    acc[i] = a[i] / 4096.0f;
    gyro[i] = g[i] / 65.5f * DEG_TO_RAD
              - (i == 0 ? GYRO_BIAS_X : i == 1 ? GYRO_BIAS_Y : GYRO_BIAS_Z);
  }
  lastImuMs = millis();
}

float accMag() { return sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]); }

void updateAttitude(float dt) {
  float m = accMag();
  if (m < 0.2f) m = 0.2f;
  float ax = -acc[0] / m, ay = -acc[1] / m;
  const float ka = 1.0f - expf(-2.0f * 3.14159f * LPF_ACCEL_HZ * dt);
  angX = (angX + gyro[1] * dt) * (1.0f - ka) + ax * ka;
  angY = (angY + gyro[0] * dt) * (1.0f - ka) + ay * ka;
  headingEst += gyro[2] * dt;
}

// ============================================================
//  3. Барометр и высота
// ============================================================
float altM = 0, altF = 0, vzF = 0, groundPa = 100500.0f;

float readPressurePa() {
#if BARO_TYPE == 1
  Wire.beginTransmission(BARO_ADDR); Wire.write(0xF7);
  if (Wire.endTransmission(false)) return -1;
  if (Wire.requestFrom(BARO_ADDR, 6) != 6) return -1;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  int32_t adc = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
  return 30000.0f + (float)adc * (80000.0f / 1048576.0f);
#elif BARO_TYPE == 2
  Wire.beginTransmission(BARO_ADDR); Wire.write(0xF6);
  if (Wire.endTransmission(false)) return -1;
  if (Wire.requestFrom(BARO_ADDR, 3) != 3) return -1;
  uint32_t up = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  return (float)up / 256.0f;
#else
  return -1;
#endif
}

void baroInit() {
#if BARO_TYPE == 1
  Wire.beginTransmission(BARO_ADDR); Wire.write(0xF4); Wire.write(0x57);
  Wire.endTransmission();
#endif
}

float pressureToAlt(float p) {
  if (p <= 0) return altF;
  return 44330.0f * (1.0f - powf(p / groundPa, 0.1903f));
}

void updateAltitude(float dt) {
  float raw = pressureToAlt(readPressurePa());
  const float k = 1.0f - expf(-dt / 0.35f);
  altF += (raw - altF) * k;
  static float prev = 0;
  float vzRaw = (altF - prev) / (dt > 0.001f ? dt : 0.001f);
  prev = altF;
  altM = altF;
  const float kv = 1.0f - expf(-dt / 0.25f);
  vzF += (vzRaw - vzF) * kv;
}

// ============================================================
//  4. Исполнительные органы
// ============================================================
Servo sP, sY, sR1, sR2, sDrogue, sMain;
float outDeg[4] = {0, 0, 0, 0}, cmdDeg[4] = {0, 0, 0, 0};
bool drogueOpen = false, mainOpen = false;

void servoInit() {
  sP.attach(PIN_SERVO_PITCH, SERVO_MIN_US, SERVO_MAX_US);
  sY.attach(PIN_SERVO_YAW, SERVO_MIN_US, SERVO_MAX_US);
  sR1.attach(PIN_SERVO_ROLL, SERVO_MIN_US, SERVO_MAX_US);
  sR2.attach(PIN_SERVO_ROLL2, SERVO_MIN_US, SERVO_MAX_US);
#if USE_SERVO_RELEASE
  sDrogue.attach(PIN_RELEASE_DROGUE, 1000, 2000);
  sDrogue.write(RELEASE_CLOSED_DEG);
#if USE_TWO_STAGE
  sMain.attach(PIN_RELEASE_MAIN, 1000, 2000);
  sMain.write(RELEASE_CLOSED_DEG);
#endif
#endif
}

void writeCanards() {
  sP.writeMicroseconds(SERVO_CENTER_US + (int)(outDeg[0] / SERVO_MAX_DEG * 400));
  sY.writeMicroseconds(SERVO_CENTER_US + (int)(outDeg[1] / SERVO_MAX_DEG * 400));
  sR1.writeMicroseconds(SERVO_CENTER_US + (int)(outDeg[2] / SERVO_MAX_DEG * 400));
  sR2.writeMicroseconds(SERVO_CENTER_US + (int)(outDeg[3] / SERVO_MAX_DEG * 400));
}

void slew(float dt) {
  const float maxStep = SERVO_RATE_DEG_S * dt;
  for (int i = 0; i < 4; i++) {
    float d = cmdDeg[i] - outDeg[i];
    if (d > maxStep) d = maxStep;
    if (d < -maxStep) d = -maxStep;
    outDeg[i] += d;
  }
  writeCanards();
}

void canardNeutral() {
  for (int i = 0; i < 4; i++) cmdDeg[i] = outDeg[i] = 0;
  writeCanards();
}

void openDrogue() {
  if (drogueOpen) return;
  drogueOpen = true;
#if USE_SERVO_RELEASE
  sDrogue.write(RELEASE_OPEN_DEG);
#endif
  beep(150);
}

void openMain() {
  if (mainOpen) return;
  mainOpen = true;
#if USE_SERVO_RELEASE && USE_TWO_STAGE
  sMain.write(RELEASE_OPEN_DEG);
#endif
  beep(400);
}

void closeBoth() {
  drogueOpen = mainOpen = false;
#if USE_SERVO_RELEASE
  sDrogue.write(RELEASE_CLOSED_DEG);
#if USE_TWO_STAGE
  sMain.write(RELEASE_CLOSED_DEG);
#endif
#endif
}

void beep(int ms) {
#if BUZZER_ON && PIN_BUZZER >= 0
  tone(PIN_BUZZER, 2200, ms);
#endif
}

// ============================================================
//  5. Режимы и контур
// ============================================================
enum Mode { M_MANUAL = 1, M_STAB = 2, M_HOLD = 3 };
Mode mode = M_STAB;
float holdAngleX = 0, holdAngleY = 0, holdHeading = 0, iTerm = 0;

void computeControl(float dt) {
  float sRoll = stick(CH_ROLL), sPitch = stick(CH_PITCH), sYaw = stick(CH_YAW);
  float uPitch, uYaw, uRoll;

  if (mode == M_MANUAL) {
    uPitch = sPitch * MANUAL_RATE_DEG_S * DEG_TO_RAD / 500.0f - KD * gyro[1];
    uYaw   = sYaw   * MANUAL_RATE_DEG_S * DEG_TO_RAD / 500.0f - KD * gyro[0];
    uRoll  = sRoll  * 0.5f * DEG_TO_RAD;
  } else if (mode == M_STAB) {
    float cmdP = -sPitch * STAB_MAX_ANGLE_DEG * DEG_TO_RAD;
    float cmdY = -sYaw   * STAB_MAX_ANGLE_DEG * DEG_TO_RAD;
    uPitch = KP * (cmdP - angX) - KD * gyro[1];
    uYaw   = KP * (cmdY - angY) - KD * gyro[0];
    uRoll  = -KD * 0.5f * gyro[2];
  } else {
    float errP = holdAngleX - angX;
    float errH = holdHeading - headingEst;
    iTerm += errH * dt * KI_HOLD;
    if (iTerm > 0.2f) iTerm = 0.2f;
    if (iTerm < -0.2f) iTerm = -0.2f;
    uPitch = KP * errP - KD * gyro[1];
    uYaw   = KP * errH + iTerm - KD * gyro[0];
    uRoll  = -KD * 0.5f * gyro[2];
  }

  float lim = SERVO_MAX_DEG * DEG_TO_RAD;
  uPitch = uPitch > lim ? lim : (uPitch < -lim ? -lim : uPitch);
  uYaw   = uYaw   > lim ? lim : (uYaw   < -lim ? -lim : uYaw);
  uRoll  = uRoll  > lim ? lim : (uRoll  < -lim ? -lim : uRoll);

  float p = uPitch * RAD_TO_DEG, y = uYaw * RAD_TO_DEG, r = uRoll * RAD_TO_DEG;
  cmdDeg[0] = p + r;
  cmdDeg[1] = y + r;
  cmdDeg[2] = -p + r;
  cmdDeg[3] = -y + r;
  for (int i = 0; i < 4; i++) {
    if (cmdDeg[i] > SERVO_MAX_DEG) cmdDeg[i] = SERVO_MAX_DEG;
    if (cmdDeg[i] < -SERVO_MAX_DEG) cmdDeg[i] = -SERVO_MAX_DEG;
  }
}

// ============================================================
//  6. Фазы полёта
// ============================================================
enum Phase { PH_IDLE, PH_ARMED, PH_BOOST, PH_COAST, PH_DEPLOYED, PH_LANDED };
Phase phase = PH_IDLE;
uint32_t tLaunch = 0, tDeploy = 0;

void flightLogic(uint32_t now) {
  if (phase == PH_ARMED) {
    static uint32_t over = 0;
    if (accMag() >= LAUNCH_ACCEL_G) {
      if (!over) over = now;
      if (now - over >= LAUNCH_HOLD_MS) {
        phase = PH_BOOST;
        tLaunch = now;
        beep(100);
      }
    } else over = 0;
    return;
  }

  if (phase == PH_BOOST || phase == PH_COAST) {
    static int neg = 0;
    if (vzF < -1.0f) neg++; else if (vzF > 0.5f) neg = 0;
    bool apogee = (neg >= APOGEE_VZ_NEG_SAMPLES) && (altM > MIN_APOGEE_ALT_M);
    bool backup = (now - tLaunch) > BACKUP_DROGUE_MS;
    if (apogee || backup) {
      openDrogue();
      phase = PH_DEPLOYED;
      tDeploy = now;
    } else if (accMag() < 1.2f && (now - tLaunch) > 800) {
      phase = PH_COAST;          // выгорание: дальше не управляем
    }
    return;
  }

  if (phase == PH_DEPLOYED) {
    if (altM < MAIN_DEPLOY_ALT_M || (now - tDeploy) > (BACKUP_MAIN_MS - BACKUP_DROGUE_MS)) openMain();
    if (altM < 5.0f && (now - tDeploy) > 6000) { phase = PH_LANDED; canardNeutral(); beep(60); }
  }
}

// ============================================================
//  7. setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ARM, INPUT);
#if PIN_BUZZER >= 0
  pinMode(PIN_BUZZER, OUTPUT);
#endif
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  imuInit();
  baroInit();
  rcInit();
  servoInit();
  canardNeutral();

  delay(200);
  float p0 = readPressurePa();
  if (p0 > 0) groundPa = p0;

  Serial.println("# t_ms,alt,vz,acc,gx,gy,gz,angX,angY,phase,mode,ch_arm,drogue,main,out0,out1,out2,out3");
  beep(60);
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastCtrl = 0, lastLog = 0, lastFast = 0, lastBaro = 0, lastMode = 0;
  float dt = 0, dtb = 0;

  rcRead();

  if (now - lastFast >= (1000 / IMU_HZ)) { lastFast = now; imuRead(); }
  if (now - lastBaro >= 20) {
    dtb = (now - lastBaro) / 1000.0f;
    lastBaro = now;
    updateAltitude(dtb);
  }

  bool alive = rcAlive();
  bool armed = alive && (ch[CH_ARM] > 1500);
  Mode newMode = (ch[CH_MODE] < 1300) ? M_MANUAL : (ch[CH_MODE] < 1700 ? M_STAB : M_HOLD);
  if (newMode != mode) {
    if (newMode == M_HOLD) { holdAngleX = angX; holdHeading = headingEst; iTerm = 0; }
    mode = newMode;
  }

  // арминг/расарминг
  if (!armed) {
    if (phase != PH_IDLE) { phase = PH_IDLE; canardNeutral(); closeBoth(); }
  } else if (phase == PH_IDLE) {
    phase = PH_ARMED;
    beep(200);
  }

  // контур
  if (now - lastCtrl >= (1000 / CTRL_HZ)) {
    dt = (now - lastCtrl) / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    lastCtrl = now;
    updateAttitude(dt);

    bool tilted = (fabs(angX) > DISABLE_TILT_DEG * DEG_TO_RAD) ||
                  (fabs(angY) > DISABLE_TILT_DEG * DEG_TO_RAD);
    bool controlWindow = (phase == PH_ARMED || phase == PH_BOOST);

    if (armed && alive && !tilted && controlWindow) computeControl(dt);
    else for (int i = 0; i < 4; i++) cmdDeg[i] = 0;

    slew(dt);
  }

  flightLogic(now);

#if LOG_SERIAL
  if (now - lastLog >= (1000 / LOG_HZ)) {
    lastLog = now;
    Serial.print(now); Serial.print(",");
    Serial.print(altM, 1); Serial.print(","); Serial.print(vzF, 1); Serial.print(",");
    Serial.print(accMag(), 2); Serial.print(",");
    Serial.print(gyro[0] * RAD_TO_DEG, 1); Serial.print(",");
    Serial.print(gyro[1] * RAD_TO_DEG, 1); Serial.print(",");
    Serial.print(gyro[2] * RAD_TO_DEG, 1); Serial.print(",");
    Serial.print(angX * RAD_TO_DEG, 1); Serial.print(",");
    Serial.print(angY * RAD_TO_DEG, 1); Serial.print(",");
    Serial.print((int)phase); Serial.print(",");
    Serial.print((int)mode); Serial.print(",");
    Serial.print(armed); Serial.print(",");
    Serial.print(drogueOpen); Serial.print(","); Serial.print(mainOpen); Serial.print(",");
    Serial.print(outDeg[0], 1); Serial.print(","); Serial.print(outDeg[1], 1); Serial.print(",");
    Serial.print(outDeg[2], 1); Serial.print(","); Serial.println(outDeg[3], 1);
  }
#endif

  if (!alive) {
    static uint32_t lastBeep = 0;
    if (now - lastBeep > 1000) { lastBeep = now; beep(300); }
  }
}
