/*

Режимы:
CALIB       - калибровка энкодера и сервоклапанов
WAIT        - сервисный режим, клапаны открыты, колено свободно
WORK        - рабочий режим: алгоритм прыжка
EMERGENCY   - авария, только из WORK

Переходы:
WAIT -> CALIB      команда CALIB
CALIB -> WAIT      команда WAIT / 0 / CALIB_EXIT
WAIT -> WORK       команда 1
WORK -> WAIT       команда 0 / WAIT
WORK -> EMERGENCY  потеря датчика/магнита, жёсткие ошибки
EMERGENCY -> WAIT  команда 0 / RESET / ACK

Алгоритм прыжка в WORK:
1. Стопа на полу: плавно открываем flex 0...70, extend = 0.
2. На носке и угол максимальный: блокируем flex = 0.
3. На носке и угол > 0: плавно открываем extend, flex = 0.
4. Касание пятки или угол = 0: блокировка до возврата на полную стопу.
5. Повтор.
*/

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <string.h>
#include <stdio.h>

// ============================================================================
// РЕЖИМЫ И ФАЗЫ ПРЫЖКА
// ============================================================================
enum SystemMode {
  MODE_CALIB,
  MODE_WAIT,
  MODE_WORK,
  MODE_EMERGENCY
};

enum JumpPhase {
  JUMP_PHASE_HOLD = 0,
  JUMP_PHASE_BEND,
  JUMP_PHASE_EXTEND
};

// ============================================================================
// SERIAL GUARD
// ============================================================================
inline bool serialReady() {
#if defined(USBCON)
  return (bool)Serial;
#else
  return true;
#endif
}

// ============================================================================
// ПОДКЛЮЧЕНИЕ
// ============================================================================
#define AS5600_I2C_ADDRESS     0x36

// Правый мотор: пин 9. Сгибание (flex / FX).
#define SERVO_PIN_R            9

// Левый мотор: пин 10. Разгибание (extend / EX).
#define SERVO_PIN_L            10

#define CALIB_BTN_PIN          2
#define LED_PIN                13
#define DIR_PIN_MAGNET         12
#define FSR_NOSE_PIN           A0
#define FSR_BACK_PIN           A1
#define TENS_POWER_PIN         4

// ============================================================================
// AS5600 / ЭНКОДЕР
// ============================================================================
#define ENCODER_INVERT         1
#define AS5600_MD_ACTIVE_HIGH  1
#define READ_PERIOD_MS         5
#define FILTER_ALPHA           0.15f

// ============================================================================
// РАБОЧИЙ ДИАПАЗОН КОЛЕНА
// ============================================================================
// 0 deg = полностью разогнуто.
// 70 deg = максимальное разрешённое сгибание.
const float KNEE_WORK_MIN_DEG      = 0.0f;
const float KNEE_WORK_MAX_DEG      = 75.0f;

// Максимальная скорость изменения угла колена.
// 140 deg/s => за 0.5 s нельзя измениться больше чем на 75 deg.
const float MAX_KNEE_RATE_DEG_PER_S = 140.0f;

// Максимальный шаг за одно чтение энкодера.
const float MAX_KNEE_STEP_DEG =
    MAX_KNEE_RATE_DEG_PER_S * ((float)READ_PERIOD_MS / 1000.0f);
    
// Допуск внутри сектора.
const float KNEE_INSIDE_TOL_DEG    = 2.0f;

// Гистерезис выхода из сектора.
const float KNEE_HYSTERESIS_DEG    = 2.0f;
const float KNEE_OUTSIDE_TOL_DEG   = KNEE_INSIDE_TOL_DEG + KNEE_HYSTERESIS_DEG;

//=============================================================================
// Жёсткий предел сгибания - ТЕСТИРОВАНИЕ
//=============================================================================
const float ANGLE_FLEX_HARD_LIMIT  = 40;

#define SAVE_CALIB_TO_EEPROM   1


// ============================================================================
// СЕРВОКЛАПАНЫ / ДЕМПФЕР
// ============================================================================
// Рабочий диапазон клапанов.
const float VALVE_NORMAL_MIN_DEG     = 0.0f;
const float VALVE_NORMAL_MAX_DEG     = 75.0f;

// Аварийный верхний предел.
const float VALVE_EMERGENCY_MAX_DEG  = 75.0f;

// Масштаб ШИМ на градус.
const float VALVE_PULSE_PER_DEG      = 10.0f;

// Жёсткие пределы импульсов для Servo library.
const int VALVE_PULSE_MIN_US         = 500;
const int VALVE_PULSE_MAX_US         = 2500;

// Мёртвая зона ШИМ.
const int VALVE_DEADBAND_US          = 5;

// Инверсия направления конкретного мотора.
const bool VALVE_INVERT_L            = false;
const bool VALVE_INVERT_R            = false;

// Шаг ручной настройки в HOME.
const int HOME_STEP_US               = 10;

// Блокировка / сильное демпфирование.
const float VALVE_LOCK_DEG           = 0.0f;

// Положения клапанов для калибровки/аварии.
float valveFlexDeg = 75.0f;   // правый клапан сгибания
float valveExtDeg  = 75.0f;   // левый клапан разгибания

// Аварийное положение: выпрямление колена.
// Левый: 75 deg, правый: 0 deg.
float valveSafeLDeg = 75.0f;
float valveSafeRDeg = 0.0f;

// Нулевые ШИМ для каждого мотора.
int32_t valveZeroPulseL = 1600;
int32_t valveZeroPulseR = 800;

/*
WAIT-режим: клапаны открыты.
*/
const float WAIT_VALVE_LEFT_OPEN_DEG  = 75.0f;
const float WAIT_VALVE_RIGHT_OPEN_DEG = 75.0f;

// ============================================================================
// FSR / СТАТУСЫ СТОПЫ
// ============================================================================
const float VCC = 5.0f;
const float R_NOSE = 1300.0f;
const float R_BACK = 3000.0f;
const bool FSR_IS_UPPER = true;
const uint8_t AVG_N = 12;
const uint32_t FSR_SAMPLE_PERIOD_MS = 20;
const uint32_t FSR_DEBOUNCE_MS = 50;

// Пороги в Омах.
// 1) Пол: оба датчика в диапазоне 800...900.
// 2) Воздух: оба показания выше 900.
// 3) На носке: носок >= 700, пятка >= 1100.
// 4) На пятке: пятка >= 700, носок >= 1100.
const float FSR_FLOOR_MIN_OHM    = 800.0f;
const float FSR_FLOOR_MAX_OHM    = 900.0f;
const float FSR_PARTIAL_MIN_OHM  = 700.0f;
const float FSR_UNLOADED_MIN_OHM = 1100.0f;

// ============================================================================
// ПРЫЖОК / НАСТРОЙКА АЛГОРИТМА
// ============================================================================
const uint32_t CONTROL_PERIOD_MS = 10;
const uint32_t SERIAL_PERIOD_MS  = 100;
const uint8_t INVALID_LIMIT = 5;

// Максимальная команда клапана в прыжке.
const float JUMP_VALVE_MAX_DEG = 75.0f;

// Скорость плавного открытия/закрытия клапанов, градусов на миллисекунду.
// 0.05 deg/ms = 50 deg/s.
// 0.08 deg/ms = 80 deg/s.
const float JUMP_FLEX_SLEW_DEG_PER_MS    = 0.05f;
const float JUMP_EXTEND_SLEW_DEG_PER_MS  = 0.08f;

// Допуск "угол максимальный".
const float JUMP_ANGLE_MAX_ON_DEG = 2.0f;

// Допуск "угол равен нулю".
const float JUMP_ANGLE_ZERO_ON_DEG = 2.0f;

// Защита от больших скачков dt, например после остановки millis() или загрузки.
const uint32_t JUMP_MAX_DT_MS = 100;

// ============================================================================
// РЕГИСТРЫ AS5600
// ============================================================================
#define REG_CONF_H             0x07
#define REG_CONF_L             0x08
#define REG_STATUS             0x0B
#define REG_RAW_ANGLE_H        0x0C
#define REG_RAW_ANGLE_L        0x0D

#define BIT_MD                 (1 << 5)
#define BIT_ML                 (1 << 4)
#define BIT_MH                 (1 << 3)

// ============================================================================
// ГЛОБАЛЬНОЕ СОСТОЯНИЕ
// ============================================================================
const uint8_t VALVE_R = 0;
const uint8_t VALVE_L = 1;

Servo servos[2];

const uint8_t valvePins[2] = {
  SERVO_PIN_R,
  SERVO_PIN_L
};

int lastPulse[2] = {1500, 1500};
float lastValveDeg[2] = {0.0f, 0.0f};

// Энкодер.
float filteredAngleDeg = 0.0f;
bool filterInitialized = false;
float zeroOffsetDeg = 0.0f;
bool encoderInvert = (ENCODER_INVERT != 0);
uint8_t lastStatus = 0;

bool inValidSector = false;
float kneeAngleDeg = 0.0f;
float holdKneeAngleDeg = 0.0f;
bool kneeSensorValid = false;
bool hasValidAngle = false;
bool sensorValid = false;
uint8_t invalidCount = 0;

// FSR / статусы стопы.
enum FootStatus {
  FOOT_STATUS_UNKNOWN = 0,
  FOOT_STATUS_AIR     = 1,
  FOOT_STATUS_FLOOR   = 2,
  FOOT_STATUS_TOE     = 3,
  FOOT_STATUS_HEEL    = 4
};

FootStatus footStatus = FOOT_STATUS_AIR;
FootStatus footPendingStatus = FOOT_STATUS_AIR;
uint32_t footPendingSinceMs = 0;

float noseResistance = -1.0f;
float backResistance = -1.0f;

// Совместимость для отладки:
// 0 = воздух,
// 1 = носок или пятка,
// 2 = полная опора.
uint8_t contactState = 0;

bool fsrInitialized = false;

// Система.
SystemMode gMode = MODE_WAIT;

// Прыжок.
JumpPhase jumpPhase = JUMP_PHASE_HOLD;
float jumpFlexCmdDeg = 0.0f;
float jumpExtendCmdDeg = 0.0f;
uint32_t lastJumpUpdateMs = 0;

// HOME внутри CALIB.
bool homingActive = false;
uint8_t homingMotor = VALVE_L;
int32_t homingPulse = 1500;

// Временные метры.
uint32_t lastReadMs = 0;
uint32_t lastFsrMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastSerialMs = 0;

// Кнопка калибровки энкодера.
bool buttonPrevRaw = false;
uint32_t lastBtnChangeMs = 0;

// Serial.
char serialCmdBuf[48];
uint8_t serialCmdLen = 0;
bool serialIntroPrinted = false;

// EEPROM layout.
const int EEPROM_MAGIC_INDEX   = 0;
const float EEPROM_MAGIC_VALUE = 123.45f;

const int EEPROM_ENC_OFFSET_INDEX = 4;
const int EEPROM_VALVE_ZERO_L_INDEX = 8;
const int EEPROM_VALVE_ZERO_R_INDEX = 12;
const int EEPROM_VALVE_F_INDEX  = 16;
const int EEPROM_VALVE_EX_INDEX = 20;

const int EEPROM_CAL2_MAGIC_INDEX  = 24;
const uint8_t EEPROM_CAL2_MAGIC_VALUE = 0x5A;
const int EEPROM_ENC_INVERT_INDEX  = 25;

// ============================================================================
// ПРОТОТИПЫ
// ============================================================================
const __FlashStringHelper* modeName(SystemMode m);
const __FlashStringHelper* jumpPhaseName(JumpPhase p);

void commandValves(float leftDeg, float rightDeg, bool emergency);
void commandWaitValves();
void commandEmergencyValves();

void readEncoder();
void readFSR(uint32_t now);

void updateJump(uint32_t now);
void resetJump();
void slewDeg(float &value, float target, float rateDegPerMs, uint32_t dtMs);

void enterEmergency(const __FlashStringHelper* reason);
void printStatus();
void processSerialCommand(const char* cmd);
void saveCalibrationToEEPROM();

const __FlashStringHelper* footStatusName(FootStatus s);
FootStatus classifyFootStatus(float noseOhm, float backOhm);
uint8_t footStatusToContactState(FootStatus s);
void updateFootStatus(FootStatus raw, uint32_t now);
void printResistance(float r);

// ============================================================================
// МАТЕМАТИЧЕСКИЕ ФУНКЦИИ
// ============================================================================
float normalizeDeg(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

float constrainf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float absf(float v) {
  return (v < 0.0f) ? -v : v;
}

int constrainInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool finitef(float v) {
  return (v == v) && (v <= 1e30f) && (v >= -1e30f);
}

bool parseFloatSimple(const char* s, float& value) {
  while (*s == ' ') s++;

  bool neg = false;
  if (*s == '-') {
    neg = true;
    s++;
  } else if (*s == '+') {
    s++;
  }

  bool hasDigits = false;
  float intPart = 0.0f;

  while (*s >= '0' && *s <= '9') {
    intPart = intPart * 10.0f + float(*s - '0');
    hasDigits = true;
    s++;
  }

  float fracPart = 0.0f;
  if (*s == '.') {
    s++;
    float mul = 0.1f;
    while (*s >= '0' && *s <= '9') {
      fracPart += float(*s - '0') * mul;
      mul *= 0.1f;
      hasDigits = true;
      s++;
    }
  }

  if (!hasDigits) return false;

  while (*s == ' ') s++;
  if (*s != '\0') return false;

  value = neg ? -(intPart + fracPart) : (intPart + fracPart);
  return true;
}

// ============================================================================
// СЕРВОКЛАПАНЫ
// ============================================================================
float constrainValveDeg(float deg, bool emergency) {
  if (!finitef(deg)) {
    deg = 0.0f;
  }

  float maxDeg = emergency ? VALVE_EMERGENCY_MAX_DEG : VALVE_NORMAL_MAX_DEG;

  if (deg < VALVE_NORMAL_MIN_DEG) deg = VALVE_NORMAL_MIN_DEG;
  if (deg > maxDeg) deg = maxDeg;

  return deg;
}

int valveDegToPulse(uint8_t motor, float deg, bool emergency) {
  float d = constrainValveDeg(deg, emergency);

  if ((motor == VALVE_L && VALVE_INVERT_L) ||
      (motor == VALVE_R && VALVE_INVERT_R)) {
    d = -d;
  }

  int32_t zeroPulse = (motor == VALVE_L) ? valveZeroPulseL : valveZeroPulseR;
  float pulseF = (float)zeroPulse + d * VALVE_PULSE_PER_DEG;

  if (!finitef(pulseF)) {
    pulseF = (float)zeroPulse;
  }

  long pulse = (long)(pulseF + (pulseF >= 0.0f ? 0.5f : -0.5f));
  return constrainInt((int)pulse, VALVE_PULSE_MIN_US, VALVE_PULSE_MAX_US);
}

void writeValve(uint8_t motor, float deg, bool emergency) {
  float cd = constrainValveDeg(deg, emergency);
  int p = valveDegToPulse(motor, cd, emergency);

  lastValveDeg[motor] = cd;

  if (abs(p - lastPulse[motor]) >= VALVE_DEADBAND_US) {
    lastPulse[motor] = p;
    servos[motor].writeMicroseconds(p);
  }
}

void writeValvePulse(uint8_t motor, int32_t pulse) {
  if (pulse < VALVE_PULSE_MIN_US) pulse = VALVE_PULSE_MIN_US;
  if (pulse > VALVE_PULSE_MAX_US) pulse = VALVE_PULSE_MAX_US;

  if (pulse != lastPulse[motor]) {
    lastPulse[motor] = (int)pulse;
    servos[motor].writeMicroseconds((int)pulse);
  }
}

void commandValves(float leftDeg, float rightDeg, bool emergency) {
  writeValve(VALVE_L, leftDeg, emergency);
  writeValve(VALVE_R, rightDeg, emergency);
}

float getWaitLeftDeg() {
  float d = (WAIT_VALVE_LEFT_OPEN_DEG >= 0.0f)
            ? WAIT_VALVE_LEFT_OPEN_DEG
            : valveFlexDeg;
  return constrainValveDeg(d, false);
}

float getWaitRightDeg() {
  float d = (WAIT_VALVE_RIGHT_OPEN_DEG >= 0.0f)
            ? WAIT_VALVE_RIGHT_OPEN_DEG
            : valveExtDeg;
  return constrainValveDeg(d, false);
}

void commandWaitValves() {
  commandValves(getWaitLeftDeg(), getWaitRightDeg(), false);
}

void commandEmergencyValves() {
  commandValves(valveSafeLDeg, valveSafeRDeg, true);
}

void printValveMotor(uint8_t motor) {
  const char* name = (motor == VALVE_L) ? "L" : "R";
  int32_t zero = (motor == VALVE_L) ? valveZeroPulseL : valveZeroPulseR;
  int32_t pulse = lastPulse[motor];

  Serial.print(F("VALVE "));
  Serial.print(name);
  Serial.print(F(" pulse="));
  Serial.print(pulse);
  Serial.print(F(" zero="));
  Serial.print(zero);
  Serial.println();
}

// ============================================================================
// НАЗВАНИЯ РЕЖИМОВ И ФАЗ ПРЫЖКА
// ============================================================================
const __FlashStringHelper* modeName(SystemMode m) {
  switch (m) {
    case MODE_CALIB:     return F("CALIB");
    case MODE_WAIT:      return F("WAIT");
    case MODE_WORK:      return F("WORK");
    case MODE_EMERGENCY: return F("EMERGENCY");
    default:             return F("UNKNOWN");
  }
}

const __FlashStringHelper* jumpPhaseName(JumpPhase p) {
  switch (p) {
    case JUMP_PHASE_HOLD:    return F("HOLD");
    case JUMP_PHASE_BEND:    return F("BEND");
    case JUMP_PHASE_EXTEND:  return F("EXTEND");
    default:                 return F("UNKNOWN");
  }
}

// ============================================================================
// ПРЫЖОК: ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================
void resetJump() {
  jumpPhase = JUMP_PHASE_HOLD;
  jumpFlexCmdDeg = 0.0f;
  jumpExtendCmdDeg = 0.0f;
  lastJumpUpdateMs = millis();
}

void slewDeg(float &value, float target, float rateDegPerMs, uint32_t dtMs) {
  if (!finitef(value)) value = 0.0f;
  if (!finitef(target)) target = 0.0f;

  target = constrainf(target, 0.0f, JUMP_VALVE_MAX_DEG);

  float delta = target - value;
  float step = rateDegPerMs * (float)dtMs;

  if (step < 0.0f) step = 0.0f;

  if (absf(delta) <= step) {
    value = target;
  } else {
    value += (delta > 0.0f) ? step : -step;
  }

  value = constrainf(value, 0.0f, JUMP_VALVE_MAX_DEG);
}

// ============================================================================
// РЕЖИМЫ: CALIB / WAIT / WORK / EMERGENCY
// ============================================================================
void printCalibHelp() {
  Serial.println(F("CALIB commands:"));
  Serial.println(F("  enc_home       set current knee angle as 0 deg"));
  Serial.println(F("  CW / CCW       encoder direction, CW = clockwise"));
  Serial.println(F("  FX  <deg>      manual right flex valve, pin 9"));
  Serial.println(F("  EX  <deg>      manual left extension valve, pin 10"));
  Serial.println(F("  HOME L         manual left valve zero setup"));
  Serial.println(F("  HOME R         manual right valve zero setup"));
  Serial.println(F("  + / -          step valve absolute PWM in HOME"));
  Serial.println(F("  SAVE           save current PWM as valve zero"));
  Serial.println(F("  HOME_EXIT      exit HOME"));
  Serial.println(F("  WAIT           exit CALIB to WAIT"));
}

void enterCalib() {
  if (gMode != MODE_WAIT) {
    Serial.println(F("CALIB ERR: only from WAIT."));
    return;
  }

  gMode = MODE_CALIB;
  homingActive = false;

  Serial.println(F("MODE: CALIB"));
  printCalibHelp();
}

void exitCalibToWait() {
  if (gMode != MODE_CALIB) {
    return;
  }

  gMode = MODE_WAIT;
  homingActive = false;
  commandWaitValves();

  Serial.println(F("MODE: WAIT"));
  Serial.println(F("Valves open. Send 1 to start WORK/JUMP."));
}

void startWork() {
  if (gMode != MODE_WAIT) {
    Serial.println(F("WORK ERR: only from WAIT."));
    return;
  }

  uint32_t now = millis();

  readEncoder();
  readFSR(now);

  if (!kneeSensorValid || !fsrInitialized || !inValidSector) {
    Serial.println(F("WORK ERR: no valid encoder/FSR data."));
    return;
  }

  gMode = MODE_WORK;
  resetJump();

  Serial.println(F("MODE: WORK"));
  Serial.print(F("Jump: "));
  Serial.println(jumpPhaseName(jumpPhase));
}

void exitWorkToWait() {
  if (gMode != MODE_WORK) {
    return;
  }

  gMode = MODE_WAIT;
  commandWaitValves();

  Serial.println(F("MODE: WAIT"));
}

void enterEmergency(const __FlashStringHelper* reason) {
  if (gMode != MODE_WORK) {
    return;
  }

  gMode = MODE_EMERGENCY;
  commandEmergencyValves();

  Serial.print(F("EMERGENCY: "));
  Serial.println(reason);
  Serial.println(F("Left valve 75 deg (extension open), right valve 0 deg (flex locked)."));
  Serial.println(F("Send 0 or RESET to acknowledge and return to WAIT."));
}

void ackEmergencyToWait() {
  if (gMode != MODE_EMERGENCY) {
    return;
  }

  gMode = MODE_WAIT;
  commandWaitValves();

  Serial.println(F("EMERGENCY cleared."));
  Serial.println(F("MODE: WAIT"));
}

// ============================================================================
// НИЗКИЙ УРОВЕНЬ I2C / AS5600
// ============================================================================
bool i2cReadReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(AS5600_I2C_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom((uint8_t)AS5600_I2C_ADDRESS, (uint8_t)1);

  if (Wire.available() != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool i2cWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AS5600_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

void initAS5600() {
  Wire.setClock(100000);

  uint8_t tmp;
  (void)i2cReadReg(REG_CONF_H, tmp);
  (void)i2cReadReg(REG_CONF_L, tmp);

  i2cWriteReg(REG_CONF_H, 0x00);
  i2cWriteReg(REG_CONF_L, 0x00);

  delay(10);
}

bool readRawAngle(uint16_t &raw) {
  uint8_t hi1, lo, hi2;

  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (!i2cReadReg(REG_RAW_ANGLE_H, hi1)) continue;
    if (!i2cReadReg(REG_RAW_ANGLE_L, lo)) continue;
    if (!i2cReadReg(REG_RAW_ANGLE_H, hi2)) continue;

    if (hi1 == hi2) {
      raw = ((uint16_t)(hi1 & 0x0F) << 8) | lo;
      return true;
    }
  }

  return false;
}

bool readEncoderAngle(float &angleDeg) {
  uint8_t status;

  if (!i2cReadReg(REG_STATUS, status)) {
    return false;
  }

  lastStatus = status;

#if AS5600_MD_ACTIVE_HIGH
  bool magnetOk = (status & BIT_MD) != 0;
#else
  bool magnetOk = (status & BIT_MD) == 0;
#endif

  bool magnetTooWeak = (status & BIT_ML) != 0;
  bool magnetTooStrong = (status & BIT_MH) != 0;

  if (!magnetOk || magnetTooWeak || magnetTooStrong) {
    return false;
  }

  uint16_t raw;
  if (!readRawAngle(raw)) {
    return false;
  }

  float deg = (float)raw * 360.0f / 4096.0f;

  if (encoderInvert) {
    deg = 360.0f - deg;
    if (deg >= 360.0f) deg -= 360.0f;
  }

  deg = normalizeDeg(deg + zeroOffsetDeg);
  angleDeg = deg;

  return true;
}

// ============================================================================
// ФИЛЬТР УГЛА
// ============================================================================
float filterAngle(float measuredDeg) {
  if (!filterInitialized) {
    filteredAngleDeg = measuredDeg;
    filterInitialized = true;
    return filteredAngleDeg;
  }

  float delta = measuredDeg - filteredAngleDeg;

  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }

  filteredAngleDeg = normalizeDeg(filteredAngleDeg + FILTER_ALPHA * delta);
  return filteredAngleDeg;
}

// ============================================================================
// СЕКТОР КОЛЕНА 0..70
// ============================================================================
bool insideCandidate(float a) {
  return (a <= KNEE_WORK_MAX_DEG + KNEE_INSIDE_TOL_DEG) ||
         (a >= 360.0f - KNEE_INSIDE_TOL_DEG);
}

bool clearlyOutside(float a) {
  return (a > KNEE_WORK_MAX_DEG + KNEE_OUTSIDE_TOL_DEG) &&
         (a < 360.0f - KNEE_OUTSIDE_TOL_DEG);
}

void updateKneeAngle(float angleDeg) {
  if (inValidSector) {
    float a = angleDeg;
  
    if (a >= 360.0f - KNEE_INSIDE_TOL_DEG) {
      a = 0.0f;
    }
  
    float desired = constrainf(a, KNEE_WORK_MIN_DEG, KNEE_WORK_MAX_DEG);
    float prev = holdKneeAngleDeg;
  
    if (desired > prev + MAX_KNEE_STEP_DEG) {
      kneeAngleDeg = prev + MAX_KNEE_STEP_DEG;
    } else if (desired < prev - MAX_KNEE_STEP_DEG) {
      kneeAngleDeg = prev - MAX_KNEE_STEP_DEG;
    } else {
      kneeAngleDeg = desired;
    }
  
    holdKneeAngleDeg = kneeAngleDeg;
    kneeSensorValid = true;
    hasValidAngle = true;
  } else {
    kneeAngleDeg = holdKneeAngleDeg;
    kneeSensorValid = false;
  }
}

// ============================================================================
// FSR / СТАТУСЫ СТОПЫ
// ============================================================================
uint16_t averageAdc(uint8_t pin) {
  analogRead(pin);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < AVG_N; ++i) {
    sum += analogRead(pin);
  }

  return static_cast<uint16_t>(sum / AVG_N);
}

float adcToVoltage(uint16_t adc) {
  return adc * (VCC / 1023.0f);
}

float fsrResistance(float voltage, float rFixed) {
  const float EPS = 0.02f;

  if (FSR_IS_UPPER) {
    if (voltage <= EPS) {
      return -1.0f;
    }

    if (voltage >= VCC - EPS) {
      return 0.0f;
    }

    return rFixed * ((VCC / voltage) - 1.0f);
  } else {
    if (voltage >= VCC - EPS) {
      return -1.0f;
    }

    if (voltage <= EPS) {
      return 0.0f;
    }

    return rFixed * (voltage / (VCC - voltage));
  }
}

const __FlashStringHelper* footStatusName(FootStatus s) {
  switch (s) {
    case FOOT_STATUS_AIR:     return F("AIR");
    case FOOT_STATUS_FLOOR:   return F("FLOOR");
    case FOOT_STATUS_TOE:     return F("TOE");
    case FOOT_STATUS_HEEL:    return F("HEEL");
    case FOOT_STATUS_UNKNOWN:
    default:                  return F("UNKNOWN");
  }
}

FootStatus classifyFootStatus(float noseOhm, float backOhm) {
  // Отрицательное значение означает обрыв / очень большое сопротивление.
  // Будем считать это "разгружено", то есть очень высоким сопротивлением.
  const float LARGE_UNLOADED_OHM = 1.0e6f;

  if (!finitef(noseOhm) || noseOhm < 0.0f) {
    noseOhm = LARGE_UNLOADED_OHM;
  }

  if (!finitef(backOhm) || backOhm < 0.0f) {
    backOhm = LARGE_UNLOADED_OHM;
  }

  // 1) Стопа на полу: оба датчика в диапазоне 800...1000.
  bool noseFloor = (noseOhm >= FSR_FLOOR_MIN_OHM && noseOhm <= FSR_FLOOR_MAX_OHM);
  bool backFloor = (backOhm >= FSR_FLOOR_MIN_OHM && backOhm <= FSR_FLOOR_MAX_OHM);

//  if (noseFloor && backFloor) {
//    return FOOT_STATUS_FLOOR;
//  }

  if (abs(noseOhm - backOhm <= 150 && (noseOhm <= 800 && backOhm <= 800))) {
    return FOOT_STATUS_FLOOR;
  }

  // 2) Все показания выше 1000 - стопа в воздухе.
//  if (noseOhm > FSR_FLOOR_MAX_OHM && backOhm > FSR_FLOOR_MAX_OHM - 100) {
//    return FOOT_STATUS_AIR;
//  }

  if (abs(noseOhm - backOhm <= 150 && (noseOhm > 800 && backOhm > 800))) {
    return FOOT_STATUS_AIR;
  }

  // 3) На носке: носок >= 700, пятка >= 1100.
  if (noseOhm <= FSR_PARTIAL_MIN_OHM && backOhm >= FSR_UNLOADED_MIN_OHM - 100) {
    return FOOT_STATUS_TOE;
  }

  // 4) На пятке: пятка >= 700, носок >= 1100.
  if (backOhm <= FSR_PARTIAL_MIN_OHM && noseOhm >= FSR_UNLOADED_MIN_OHM - 100) {
    return FOOT_STATUS_HEEL;
  }

  // Промежуточные/непонятные комбинации не меняем стабильный статус.
  return FOOT_STATUS_UNKNOWN;
}

uint8_t footStatusToContactState(FootStatus s) {
  switch (s) {
    case FOOT_STATUS_FLOOR:
      return 2;

    case FOOT_STATUS_TOE:
    case FOOT_STATUS_HEEL:
      return 1;

    case FOOT_STATUS_AIR:
    case FOOT_STATUS_UNKNOWN:
    default:
      return 0;
  }
}

void updateFootStatus(FootStatus raw, uint32_t now) {
  // Если классификация не определена, не дёргаем стабильный статус.
  if (raw == FOOT_STATUS_UNKNOWN) {
    footPendingStatus = footStatus;
    footPendingSinceMs = now;
    contactState = footStatusToContactState(footStatus);
    return;
  }

  // Если сырой статус совпадает со стабильным, сбрасываем таймер ожидания.
  if (raw == footStatus) {
    footPendingStatus = raw;
    footPendingSinceMs = now;
    contactState = footStatusToContactState(footStatus);
    return;
  }

  // Если появился новый кандидат на смену статуса, запускаем дебаунс.
  if (raw != footPendingStatus) {
    footPendingStatus = raw;
    footPendingSinceMs = now;
    contactState = footStatusToContactState(footStatus);
    return;
  }

  // Кандидат держится достаточно долго - фиксируем новый статус.
  if ((now - footPendingSinceMs) >= FSR_DEBOUNCE_MS) {
    footStatus = raw;
  }

  contactState = footStatusToContactState(footStatus);
}

void printResistance(float r) {
  if (r < 0.0f) {
    Serial.print(F("inf"));
  } else {
    Serial.print(r, 0);
  }
}

// ============================================================================
// ЧТЕНИЕ ДАТЧИКОВ
// ============================================================================
void readEncoder() {
  uint32_t now = millis();
  lastReadMs = now;

  float measured = 0.0f;

  if (readEncoderAngle(measured)) {
    measured = filterAngle(measured);
    updateKneeAngle(measured);

    if (kneeSensorValid) {
      invalidCount = 0;
    } else {
      invalidCount++;
    }
  } else {
    kneeSensorValid = false;
    invalidCount++;
  }

  if (invalidCount >= INVALID_LIMIT) {
    sensorValid = false;

    // Авария возможна только из WORK.
    if (gMode == MODE_WORK) {
      enterEmergency(F("encoder/magnet invalid"));
    }
  } else {
    sensorValid = hasValidAngle;
  }

  digitalWrite(LED_PIN, sensorValid ? HIGH : LOW);
}

void readFSR(uint32_t now) {
  lastFsrMs = now;

  uint16_t adcN = averageAdc(FSR_NOSE_PIN);
  uint16_t adcB = averageAdc(FSR_BACK_PIN);

  float vN = adcToVoltage(adcN);
  float vB = adcToVoltage(adcB);

  noseResistance = fsrResistance(vN, R_NOSE);
  backResistance = fsrResistance(vB, R_BACK);

  FootStatus rawStatus = classifyFootStatus(noseResistance, backResistance);
  updateFootStatus(rawStatus, now);

  fsrInitialized = true;
}

// ============================================================================
// АЛГОРИТМ ПРЫЖКА
// ============================================================================
void updateJump(uint32_t now) {
  if (gMode != MODE_WORK) {
    return;
  }

  if (!kneeSensorValid || !fsrInitialized || !inValidSector) {
    enterEmergency(F("sensor lost"));
    return;
  }

  uint32_t dt = now - lastJumpUpdateMs;
  if (dt > JUMP_MAX_DT_MS) {
    dt = JUMP_MAX_DT_MS;
  }

  lastJumpUpdateMs = now;

  // --------------------------------------------------------------------------
  // Переходы между фазами прыжка.
  // --------------------------------------------------------------------------

  if (jumpPhase == JUMP_PHASE_BEND) {
    bool maxReached = (kneeAngleDeg >= (ANGLE_FLEX_HARD_LIMIT - JUMP_ANGLE_MAX_ON_DEG));

    // 2. Как только мы на носке и угол максимальный - блокируем flex = 0.
    if (footStatus == FOOT_STATUS_TOE && maxReached) {
      jumpFlexCmdDeg = 0.0f;
      jumpExtendCmdDeg = 0.0f;
      jumpPhase = JUMP_PHASE_EXTEND;
    }
    // Если стопа неожиданно покинула пол/носок - уходим в безопасное удержание.
    else if (footStatus != FOOT_STATUS_FLOOR && footStatus != FOOT_STATUS_TOE) {
      jumpPhase = JUMP_PHASE_HOLD;
    }
  }
  else if (jumpPhase == JUMP_PHASE_EXTEND) {
    // 4. Касаемся пятки ИЛИ угол = 0 - блокировка до возврата на полную стопу.
    if (footStatus == FOOT_STATUS_HEEL || kneeAngleDeg <= JUMP_ANGLE_ZERO_ON_DEG) {
      jumpPhase = JUMP_PHASE_HOLD;
    }
    // Если потеряли носок - тоже в удержание.
    else if (footStatus != FOOT_STATUS_TOE) {
      jumpPhase = JUMP_PHASE_HOLD;
    }
  }
  else {
    // JUMP_PHASE_HOLD:
    // 5. Ждём полную стопу и повторяем.
    if (footStatus == FOOT_STATUS_TOE) {
//       if (footStatus == FOOT_STATUS_FLOOR) {
      jumpFlexCmdDeg = 0.0f;
      jumpExtendCmdDeg = 0.0f;
      jumpPhase = JUMP_PHASE_BEND;
    }
  }

  // --------------------------------------------------------------------------
  // Целевые команды клапанов для текущей фазы.
  // --------------------------------------------------------------------------

  float targetFlex = 0.0f;
  float targetExtend = 0.0f;

  if (jumpPhase == JUMP_PHASE_BEND) {
    // 1. Стопа на полу: плавно открываем flex 0...70, extend = 0.
    targetFlex = JUMP_VALVE_MAX_DEG;
    targetExtend = 0.0f;
  }
  else if (jumpPhase == JUMP_PHASE_EXTEND) {
    // 3. Мы на носке и угол > 0: плавно увеличиваем extend, flex = 0.
    targetFlex = 0.0f;
    targetExtend = JUMP_VALVE_MAX_DEG;
  }
  else {
    // HOLD: безопасно держим оба клапана закрытыми/заблокированными.
    targetFlex = 0.0f;
    targetExtend = 0.0f;
  }

  // --------------------------------------------------------------------------
  // Плавное изменение команд.
  // --------------------------------------------------------------------------

  slewDeg(jumpFlexCmdDeg, targetFlex, JUMP_FLEX_SLEW_DEG_PER_MS, dt);
  slewDeg(jumpExtendCmdDeg, targetExtend, JUMP_EXTEND_SLEW_DEG_PER_MS, dt);

  // ВАЖНО:
  // commandValves(leftDeg, rightDeg)
  // left  = extend, пин 10
  // right = flex,   пин 9
  commandValves(jumpExtendCmdDeg, jumpFlexCmdDeg, false);
}

// ============================================================================
// EEPROM / КАЛИБРОВКА
// ============================================================================
bool loadCalibrationFromEEPROM() {
  float magic;
  EEPROM.get(EEPROM_MAGIC_INDEX, magic);

  if (absf(magic - EEPROM_MAGIC_VALUE) > 0.01f) {
    return false;
  }

  EEPROM.get(EEPROM_ENC_OFFSET_INDEX, zeroOffsetDeg);

  if (!(zeroOffsetDeg >= 0.0f && zeroOffsetDeg < 360.0f)) {
    zeroOffsetDeg = 0.0f;
  }

  zeroOffsetDeg = normalizeDeg(zeroOffsetDeg);

  EEPROM.get(EEPROM_VALVE_ZERO_L_INDEX, valveZeroPulseL);
  EEPROM.get(EEPROM_VALVE_ZERO_R_INDEX, valveZeroPulseR);
  EEPROM.get(EEPROM_VALVE_F_INDEX, valveFlexDeg);
  EEPROM.get(EEPROM_VALVE_EX_INDEX, valveExtDeg);

  uint8_t cal2Magic = 0;
  EEPROM.get(EEPROM_CAL2_MAGIC_INDEX, cal2Magic);

  if (cal2Magic == EEPROM_CAL2_MAGIC_VALUE) {
    uint8_t inv = 0;
    EEPROM.get(EEPROM_ENC_INVERT_INDEX, inv);
    encoderInvert = (inv != 0);
  } else {
    encoderInvert = (ENCODER_INVERT != 0);
  }

  valveZeroPulseL = constrainInt(valveZeroPulseL, VALVE_PULSE_MIN_US, VALVE_PULSE_MAX_US);
  valveZeroPulseR = constrainInt(valveZeroPulseR, VALVE_PULSE_MIN_US, VALVE_PULSE_MAX_US);

  // Миграция со старой шкалы:
  // раньше open = 0, теперь open = 70.
  if (!(valveFlexDeg >= 0.0f && valveFlexDeg <= VALVE_NORMAL_MAX_DEG) ||
      valveFlexDeg < 1.0f) {
    valveFlexDeg = VALVE_NORMAL_MAX_DEG;
  }

  if (!(valveExtDeg >= 0.0f && valveExtDeg <= VALVE_NORMAL_MAX_DEG) ||
      valveExtDeg < 1.0f) {
    valveExtDeg = VALVE_NORMAL_MAX_DEG;
  }

  valveFlexDeg = constrainValveDeg(valveFlexDeg, false);
  valveExtDeg  = constrainValveDeg(valveExtDeg, false);

  return true;
}

void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_MAGIC_INDEX, EEPROM_MAGIC_VALUE);
  EEPROM.put(EEPROM_ENC_OFFSET_INDEX, zeroOffsetDeg);
  EEPROM.put(EEPROM_VALVE_ZERO_L_INDEX, valveZeroPulseL);
  EEPROM.put(EEPROM_VALVE_ZERO_R_INDEX, valveZeroPulseR);
  EEPROM.put(EEPROM_VALVE_F_INDEX, valveFlexDeg);
  EEPROM.put(EEPROM_VALVE_EX_INDEX, valveExtDeg);
  EEPROM.put(EEPROM_CAL2_MAGIC_INDEX, EEPROM_CAL2_MAGIC_VALUE);
  EEPROM.put(EEPROM_ENC_INVERT_INDEX, (uint8_t)(encoderInvert ? 1 : 0));
}

void calibrateZeroToCurrent() {
  if (gMode != MODE_CALIB) {
    Serial.println(F("CAL ERR: enc_home only in CALIB."));
    return;
  }

  if (!filterInitialized) {
    Serial.println(F("CAL: no valid sensor data yet."));
    return;
  }

  zeroOffsetDeg = normalizeDeg(zeroOffsetDeg - filteredAngleDeg);

  filteredAngleDeg = 0.0f;
  filterInitialized = true;

  inValidSector = true;
  kneeAngleDeg = 0.0f;
  holdKneeAngleDeg = 0.0f;

  kneeSensorValid = true;
  hasValidAngle = true;
  sensorValid = true;
  invalidCount = 0;

#if SAVE_CALIB_TO_EEPROM
  saveCalibrationToEEPROM();
#endif

  Serial.println(F("CAL: current knee position set to 0 deg."));
}

void setEncoderInvert(bool inv) {
  if (gMode != MODE_CALIB) {
    Serial.println(F("ENC ERR: CW/CCW only in CALIB."));
    return;
  }

  if (encoderInvert == inv) {
    return;
  }

  encoderInvert = inv;

  filterInitialized = false;
  inValidSector = false;

  kneeSensorValid = false;
  hasValidAngle = false;
  sensorValid = false;
  invalidCount = INVALID_LIMIT;

#if SAVE_CALIB_TO_EEPROM
  saveCalibrationToEEPROM();
#endif

  Serial.print(F("ENC dir: "));
  Serial.println(inv ? F("CW/inverted") : F("CCW/normal"));
}

void handleCalibrationButton() {
  if (gMode != MODE_CALIB) {
    return;
  }

  bool rawPressed = (digitalRead(CALIB_BTN_PIN) == LOW);
  uint32_t now = millis();

  if (rawPressed != buttonPrevRaw) {
    buttonPrevRaw = rawPressed;
    lastBtnChangeMs = now;
  }

  static bool stablePressed = false;

  if ((now - lastBtnChangeMs) >= 50) {
    if (buttonPrevRaw != stablePressed) {
      stablePressed = buttonPrevRaw;

      if (stablePressed) {
        calibrateZeroToCurrent();
      }
    }
  }
}

// ============================================================================
// SERIAL / ОТЛАДКА
// ============================================================================
void printStatus() {
  if (!serialReady()) {
    return;
  }

  Serial.print(F("Mode="));
  Serial.print(modeName(gMode));

  Serial.print(F(" Jump="));
  Serial.print(jumpPhaseName(jumpPhase));

  Serial.print(F(" Ang="));
  Serial.print(kneeAngleDeg, 1);

  Serial.print(F(" Fil="));
  Serial.print(filteredAngleDeg, 1);

  Serial.print(F(" FS="));
  Serial.print(footStatusName(footStatus));

  Serial.print(F(" C="));
  Serial.print(contactState);

  Serial.print(F(" N="));
  printResistance(noseResistance);

  Serial.print(F(" B="));
  printResistance(backResistance);

  Serial.print(F(" JFL="));
  Serial.print(jumpFlexCmdDeg, 1);

  Serial.print(F(" JEX="));
  Serial.print(jumpExtendCmdDeg, 1);

  Serial.print(F(" VL="));
  Serial.print(lastValveDeg[VALVE_L], 1);

  Serial.print(F(" VR="));
  Serial.print(lastValveDeg[VALVE_R], 1);

  Serial.print(F(" PL="));
  Serial.print(lastPulse[VALVE_L]);

  Serial.print(F(" PR="));
  Serial.print(lastPulse[VALVE_R]);

  Serial.print(F(" ZL="));
  Serial.print(valveZeroPulseL);

  Serial.print(F(" ZR="));
  Serial.print(valveZeroPulseR);

  Serial.print(F(" F="));
  Serial.print(valveFlexDeg, 1);

  Serial.print(F(" EX="));
  Serial.print(valveExtDeg, 1);

  Serial.print(F(" Valid="));
  Serial.print(sensorValid ? 1 : 0);

  Serial.print(F(" Sector="));
  Serial.print(inValidSector ? 1 : 0);

  Serial.print(F(" InvCnt="));
  Serial.print(invalidCount);

  Serial.print(F(" S=0x"));
  Serial.print(lastStatus, HEX);

  Serial.print(F(" Off="));
  Serial.print(zeroOffsetDeg, 1);

  if (homingActive) {
    Serial.print(F(" HOME="));
    Serial.print(homingMotor == VALVE_L ? F("L") : F("R"));
  }

  Serial.println();
}

void processSerialCommand(const char* cmd) {
  if (strlen(cmd) == 0) {
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  // --------------------------------------------------------------------------
  // CALIB
  // --------------------------------------------------------------------------
  if (gMode == MODE_CALIB) {
    if (strcmp(cmd, "WAIT") == 0 ||
        strcmp(cmd, "0") == 0 ||
        strcmp(cmd, "CALIB_EXIT") == 0 ||
        strcmp(cmd, "EXIT") == 0) {
      exitCalibToWait();
      return;
    }

    if (strcmp(cmd, "CALIB") == 0) {
      Serial.println(F("CALIB: already in CALIB."));
      return;
    }

    if (homingActive) {
      if (strcmp(cmd, "+") == 0 || strcmp(cmd, "-") == 0) {
        int step = (cmd[0] == '+') ? HOME_STEP_US : -HOME_STEP_US;

        homingPulse += step;

        if (homingPulse < VALVE_PULSE_MIN_US) homingPulse = VALVE_PULSE_MIN_US;
        if (homingPulse > VALVE_PULSE_MAX_US) homingPulse = VALVE_PULSE_MAX_US;

        writeValvePulse(homingMotor, homingPulse);
        printValveMotor(homingMotor);
        return;
      }

      if (strcmp(cmd, "SAVE") == 0) {
        if (homingMotor == VALVE_L) {
          valveZeroPulseL = homingPulse;
        } else {
          valveZeroPulseR = homingPulse;
        }

        lastPulse[homingMotor] = (int)homingPulse;
        lastValveDeg[homingMotor] = 0.0f;

#if SAVE_CALIB_TO_EEPROM
        saveCalibrationToEEPROM();
#endif

        Serial.println(F("SAVE: absolute PWM stored as zero PWM."));
        printValveMotor(homingMotor);
        return;
      }

      if (strcmp(cmd, "HOME_EXIT") == 0) {
        homingActive = false;
        Serial.println(F("HOME exited."));
        return;
      }

      Serial.println(F("CALIB/HOME: use +, -, SAVE, HOME_EXIT, status."));
      return;
    }

    if (strcmp(cmd, "enc_home") == 0) {
      calibrateZeroToCurrent();
      return;
    }

    if (strcmp(cmd, "CW") == 0) {
      setEncoderInvert(true);
      return;
    }

    if (strcmp(cmd, "CCW") == 0) {
      setEncoderInvert(false);
      return;
    }

    if (strcmp(cmd, "HOME") == 0) {
      Serial.println(F("Usage: HOME L or HOME R"));
      Serial.println(F("Then: +, -, SAVE, HOME_EXIT"));
      return;
    }

    if (strncmp(cmd, "HOME ", 5) == 0) {
      const char* arg = cmd + 5;
      while (*arg == ' ') arg++;

      if (strcmp(arg, "L") == 0) {
        homingActive = true;
        homingMotor = VALVE_L;
        homingPulse = lastPulse[VALVE_L];

        if (homingPulse < VALVE_PULSE_MIN_US || homingPulse > VALVE_PULSE_MAX_US) {
          homingPulse = 1500;
        }

        Serial.println(F("VALVE_HOME: left motor, pin 10 (extension)"));
        Serial.println(F("Use + / - to change absolute PWM."));
        printValveMotor(VALVE_L);
      } else if (strcmp(arg, "R") == 0) {
        homingActive = true;
        homingMotor = VALVE_R;
        homingPulse = lastPulse[VALVE_R];

        if (homingPulse < VALVE_PULSE_MIN_US || homingPulse > VALVE_PULSE_MAX_US) {
          homingPulse = 1500;
        }

        Serial.println(F("VALVE_HOME: right motor, pin 9 (flex)"));
        Serial.println(F("Use + / - to change absolute PWM."));
        printValveMotor(VALVE_R);
      } else {
        Serial.println(F("VALVE_HOME ERR: use L or R."));
      }

      return;
    }

    if (cmd[0] == 'F' && cmd[1] == 'X' && cmd[2] == ' ' && cmd[3] != '\0') {
      float v;

      if (parseFloatSimple(cmd + 3, v)) {
        float d = constrainValveDeg(v, false);
        writeValve(VALVE_R, d, false);

        Serial.print(F("FX manual: right flex valve = "));
        Serial.print(d, 1);
        Serial.println(F(" deg"));

        printValveMotor(VALVE_R);
      } else {
        Serial.println(F("FX ERR: usage FX <deg>"));
      }

      return;
    }

    if (cmd[0] == 'E' && cmd[1] == 'X' && cmd[2] == ' ' && cmd[3] != '\0') {
      float v;

      if (parseFloatSimple(cmd + 3, v)) {
        float d = constrainValveDeg(v, false);
        writeValve(VALVE_L, d, false);

        Serial.print(F("EX manual: left extension valve = "));
        Serial.print(d, 1);
        Serial.println(F(" deg"));

        printValveMotor(VALVE_L);
      } else {
        Serial.println(F("EX ERR: usage EX <deg>"));
      }

      return;
    }

    if (strcmp(cmd, "1") == 0) {
      Serial.println(F("WORK ERR: exit CALIB to WAIT first."));
      return;
    }

    Serial.print(F("CALIB unknown command: "));
    Serial.println(cmd);
    return;
  }

  // --------------------------------------------------------------------------
  // WAIT
  // --------------------------------------------------------------------------
  if (gMode == MODE_WAIT) {
    if (strcmp(cmd, "1") == 0 || strcmp(cmd, "WORK") == 0) {
      startWork();
      return;
    }

    if (strcmp(cmd, "CALIB") == 0) {
      enterCalib();
      return;
    }

    if (strcmp(cmd, "0") == 0) {
      Serial.println(F("WAIT: already in service mode."));
      return;
    }

    Serial.print(F("WAIT unknown command: "));
    Serial.println(cmd);
    return;
  }

  // --------------------------------------------------------------------------
  // WORK
  // --------------------------------------------------------------------------
  if (gMode == MODE_WORK) {
    if (strcmp(cmd, "0") == 0 || strcmp(cmd, "WAIT") == 0) {
      exitWorkToWait();
      return;
    }

    if (strcmp(cmd, "1") == 0) {
      Serial.println(F("WORK: already running."));
      return;
    }

    if (strcmp(cmd, "CALIB") == 0) {
      Serial.println(F("CALIB ERR: exit WORK to WAIT first."));
      return;
    }

    Serial.print(F("WORK unknown command: "));
    Serial.println(cmd);
    return;
  }

  // --------------------------------------------------------------------------
  // EMERGENCY
  // --------------------------------------------------------------------------
  if (gMode == MODE_EMERGENCY) {
    if (strcmp(cmd, "0") == 0 ||
        strcmp(cmd, "RESET") == 0 ||
        strcmp(cmd, "ACK") == 0) {
      ackEmergencyToWait();
      return;
    }

    if (strcmp(cmd, "1") == 0) {
      Serial.println(F("EMERGENCY: send 0 or RESET first."));
      return;
    }

    Serial.print(F("EMERGENCY unknown command: "));
    Serial.println(cmd);
    return;
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        processSerialCommand(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else if (c >= ' ') {
      // Мгновенная реакция на одиночные '1' и '0'.
      if (serialCmdLen == 0 && (c == '1' || c == '0')) {
        char tmp[2];
        tmp[0] = c;
        tmp[1] = '\0';
        processSerialCommand(tmp);
      } else {
        if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
          serialCmdBuf[serialCmdLen++] = c;
        }
      }
    }
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  pinMode(CALIB_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIR_PIN_MAGNET, OUTPUT);
  digitalWrite(DIR_PIN_MAGNET, LOW);

  Serial.begin(115200);
  Wire.begin();

  pinMode(TENS_POWER_PIN, OUTPUT);
  digitalWrite(TENS_POWER_PIN, HIGH);

  initAS5600();

#if SAVE_CALIB_TO_EEPROM
  if (!loadCalibrationFromEEPROM()) {
    saveCalibrationToEEPROM();
  }
#endif

  for (uint8_t i = 0; i < 2; ++i) {
    servos[i].attach(valvePins[i], VALVE_PULSE_MIN_US, VALVE_PULSE_MAX_US);
  }

  // Принудительно отправим первый ШИМ.
  lastPulse[VALVE_L] = -10000;
  lastPulse[VALVE_R] = -10000;

  gMode = MODE_WAIT;
  commandWaitValves();
  resetJump();

  serialIntroPrinted = false;
}

void loop() {
  uint32_t now = millis();

  if (!serialIntroPrinted && serialReady()) {
    serialIntroPrinted = true;

    Serial.println(F("=== Knee jump minimal 0.7 ==="));

#if SAVE_CALIB_TO_EEPROM
    Serial.print(F("CAL: encoder offset = "));
    Serial.println(zeroOffsetDeg, 2);

    Serial.print(F("CAL: encoder dir = "));
    Serial.println(encoderInvert ? F("CW/inverted") : F("CCW/normal"));

    Serial.print(F("CAL: valve zero L = "));
    Serial.println(valveZeroPulseL);

    Serial.print(F("CAL: valve zero R = "));
    Serial.println(valveZeroPulseR);

    Serial.print(F("CAL: valve F = "));
    Serial.println(valveFlexDeg, 2);

    Serial.print(F("CAL: valve EX = "));
    Serial.println(valveExtDeg, 2);
#endif

    Serial.println(F("Modes: CALIB, WAIT, WORK, EMERGENCY"));
    Serial.println(F("Start mode: WAIT"));
    Serial.println(F("Valves are open in WAIT."));
    Serial.println(F("Send 1 to start WORK/JUMP."));
    Serial.println(F("Send CALIB to enter calibration."));
    Serial.println(F("Send 0 in WORK to return to WAIT."));
    Serial.println();
  }

  handleCalibrationButton();
  handleSerialCommands();

  if (now - lastReadMs >= READ_PERIOD_MS) {
    readEncoder();
  }

  if (now - lastFsrMs >= FSR_SAMPLE_PERIOD_MS) {
    readFSR(now);
  }

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;

    if (gMode == MODE_WORK) {
      updateJump(now);
    } else if (gMode == MODE_WAIT) {
      commandWaitValves();
    } else if (gMode == MODE_EMERGENCY) {
      commandEmergencyValves();
    }

    // В CALIB автоматическое управление клапанами не выполняется,
    // чтобы не мешать ручной настройке.
  }

  if (now - lastSerialMs >= SERIAL_PERIOD_MS) {
    lastSerialMs = now;
    printStatus();
  }
}
