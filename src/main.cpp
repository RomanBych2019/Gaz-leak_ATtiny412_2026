#include <Arduino.h>

#ifndef AUTOCALIBR
#define AUTOCALIBR 1
#endif

#if AUTOCALIBR
#include <EEPROM.h>
#endif

// ============================================================
// Контроллер датчика газа на ATtiny412
// ============================================================
// PA0 - UPDI, не использовать как GPIO
// PA1 - светодиод состояния платы                / LIN TX в версии LIN_DRIVER
// PA2 - вход АЦП датчика                         / LIN RX в версии LIN_DRIVER
// PA3 - отладочный TX, программный UART 9600 бод / светодиод состояния платы в версии LIN_DRIVER
// PA6 - выход силового ключа 2                   / вход АЦП датчика в версии LIN_DRIVER
// PA7 - выход силового ключа 1                   / единственный активный выход в версии LIN_DRIVER
//
// Логика:
// - напряжение датчика проверяется по заводским или откалиброванным порогам АЦП
// - АЦП около 0 В: датчик оборван или линия датчика подключена к GND
// - АЦП около +5 В: выход датчика замкнут на VCC
// - состояние ALARM защелкивается в RAM и сбрасывается только отключением питания
// - остальные состояния могут вернуться в NORMAL, когда АЦП вернется в допустимый диапазон
// ============================================================

#ifndef DEBUG
#define DEBUG 0
#endif

#ifndef LIN_DRIVER
#define LIN_DRIVER 0
#endif

#if !LIN_DRIVER
#define LED_PIN       PIN_PA1
#define SENSOR_PIN    PIN_PA2
#define DEBUG_TX_PIN  PIN_PA3
#define KEY1_PIN      PIN_PA7
#define KEY2_PIN      PIN_PA6
#else
#define LED_PIN       PIN_PA3
#define SENSOR_PIN    PIN_PA6
#define DEBUG_TX_PIN  PIN_PA3
#define KEY_PIN       PIN_PA7
#define LIN_RX_PIN    PIN_PA2
#define LIN_TX_PIN    PIN_PA1
#endif

#if DEBUG && LIN_DRIVER
// В LIN-версии PA3 используется светодиодом, но при DEBUG приоритет у
// отладочного TX на PA3, поэтому индикация светодиодом отключается.
#define LED_ENABLED   0
#else
#define LED_ENABLED   1
#endif

// ============================================================
// Отладочный программный UART на PA3
// ============================================================

#if DEBUG

const uint32_t  DEBUG_BAUD = 2400;
const uint16_t  BIT_TIME_US = (1000000UL / DEBUG_BAUD);

void debugTxWriteBit(bool bitValue)
{
  digitalWrite(DEBUG_TX_PIN, bitValue ? HIGH : LOW);
  delayMicroseconds(BIT_TIME_US);
}

void debugTxWriteByte(uint8_t b)
{
  noInterrupts();

  // Стартовый бит
  debugTxWriteBit(false);

  // Биты данных, младший бит первым
  for (uint8_t i = 0; i < 8; i++) {
    debugTxWriteBit(b & 0x01);
    b >>= 1;
  }

  // Стоповый бит
  debugTxWriteBit(true);

  interrupts();
}

void debugPrint(const char *s)
{
  while (*s) {
    debugTxWriteByte((uint8_t)*s++);
  }
}

void debugPrintln(const char *s)
{
  debugPrint(s);
  debugPrint("\r\n");
}

void debugPrintUint(uint32_t value)
{
  char buf[12];
  ultoa(value, buf, 10);
  debugPrint(buf);
}
#endif

// ============================================================
// Основные настройки
// ============================================================

// АЦП в Arduino analogRead() 10-битный: 0...1023
const uint16_t ADC_MAX_VALUE = 1023;

// Время прогрева датчика после включения питания.
const uint32_t SENSOR_WARMUP_MS = 45000UL;

// Период отладочного вывода
const uint32_t DEBUG_PERIOD_MS = 5000UL;

// Время подтверждения тревоги по газу.
// ALARM защелкивается, только если газовый порог непрерывно превышен
// в течение этого времени.
const uint32_t ALARM_CONFIRM_MS = 3000UL;

// Время подтверждения неисправности датчика по мин./макс. напряжению
const uint32_t FAULT_CONFIRM_MS = 1000UL;

// Время выхода из неисправности датчика.
// Состояния FAULT могут вернуться в NORMAL, когда АЦП вернется в допустимый диапазон.
const uint32_t FAULT_RELEASE_MS = 1000UL;

// Усреднение АЦП
const uint8_t ADC_SAMPLES = 16;

// Коэффициент ФНЧ для АЦП.
// 8 означает: filtered = filtered * 7/8 + raw * 1/8
const uint8_t FILTER_DIV = 8;

// ============================================================
// Газовые пороги
// ============================================================

// true  - сигнал АЦП увеличивается при наличии газа
// false - сигнал АЦП уменьшается при наличии газа
const bool SENSOR_SIGNAL_INCREASES_WITH_GAS = true;

// Порог включения тревоги по газу без автокалибровки
const uint16_t GAS_ALARM_ON_ADC = 650;

// Базовый уровень чистого воздуха для заводских порогов.
// При автокалибровке пороги сдвигаются на разницу между измеренным
// чистым воздухом и этим значением.
// 0.3 В при VCC = 5 В: 0.3 / 5.0 * 1023 ~= 61.
const uint16_t GAS_CLEAN_AIR_FACTORY_ADC = 61;

// Порог выключения тревоги по газу больше не используется для сброса ALARM,
// потому что ALARM защелкивается до отключения питания.
// Оставлен только для возможных будущих вариантов без защелкивания.
const uint16_t GAS_ALARM_OFF_ADC = 580;

#if AUTOCALIBR
// Максимально допустимая разница между новой калибровкой и сохраненной.
// Если разница больше, считаем, что запуск выполнен в газовой среде,
// и используем значение из EEPROM.
const uint16_t AUTOCALIBR_MAX_DELTA_ADC = 100;

const uint16_t AUTOCALIBR_EEPROM_SIGNATURE = 0x4743; // 'G''C'
const uint8_t AUTOCALIBR_EEPROM_ADDR = 0;
#endif

// ============================================================
// Диагностические пороги напряжения датчика
// ============================================================
//
// На входе датчика SENSOR_PIN установлена внешняя подтяжка 4.7 кОм к GND.
//
// ADC <= ADC_SENSOR_OPEN_OR_GND_ON:
//   датчик оборван или линия датчика подключена к GND.
//   напряжение на SENSOR_PIN около 0 В.
//
// ADC >= ADC_SENSOR_SHORT_TO_VCC_ON:
//   выход датчика замкнут на питание.
//   напряжение на SENSOR_PIN около +5 В.
//
// Пороги OFF задают гистерезис для возврата из состояний неисправности.

const uint16_t ADC_SENSOR_OPEN_OR_GND_ON  = 20;
const uint16_t ADC_SENSOR_OPEN_OR_GND_OFF = 50;

const uint16_t ADC_SENSOR_SHORT_TO_VCC_ON  = 1000;
const uint16_t ADC_SENSOR_SHORT_TO_VCC_OFF = 970;

// Приблизительные значения при VCC = 5 В:
// ADC 20   ~= 0.10 V
// ADC 50   ~= 0.24 V
// ADC 970  ~= 4.74 V
// ADC 1000 ~= 4.89 V

// ============================================================
// LIN interface
// ============================================================
//
// TLIN2029ADRBRQ1 - это физический LIN-трансивер. Протокол LIN
// обслуживает ATtiny412 через USART0:
// - TXD трансивера <- PA1 / LIN_TX_PIN
// - RXD трансивера -> PA2 / LIN_RX_PIN
//
// Узел работает как LIN slave и отвечает на header с PID 0x92
// (LIN ID 0x12) статусным кадром:
// byte 0: SystemState
// byte 1: flags, bit0=alarmLatched, bit1=outputActive, bit2=fault, bit3=warmup
// byte 2: filteredAdc low byte
// byte 3: filteredAdc high byte

#if LIN_DRIVER
const uint32_t LIN_BAUD = 19200UL;
const uint8_t LIN_STATUS_ID = 0x12;
const uint8_t LIN_STATUS_DATA_LEN = 4;
const uint8_t LIN_HEADER_TIMEOUT_MS = 10;

enum LinRxState {
  LIN_WAIT_SYNC,
  LIN_WAIT_PID
};

LinRxState linRxState = LIN_WAIT_SYNC;
uint32_t lastLinByteMs = 0;
#endif

// ============================================================
// Состояния системы
// ============================================================

enum SystemState {
  STATE_WARMUP,               // датчик прогревается после включения питания
  STATE_NORMAL,               // датчик исправен, газовый порог не превышен     
  STATE_ALARM,                // датчик исправен, газовый порог превышен
  STATE_SENSOR_OPEN_OR_GND,   // датчик оборван
  STATE_SENSOR_SHORT_TO_VCC   // выход датчика замкнут на питание
};

SystemState systemState = STATE_WARMUP;

uint16_t filteredAdc = 0;
uint16_t activeGasAlarmOnAdc = GAS_ALARM_ON_ADC;
uint16_t activeGasAlarmOffAdc = GAS_ALARM_OFF_ADC;

// Защелка ALARM. Хранится в RAM, поэтому сбрасывается после отключения питания.
// Примечание: аппаратный сброс или сброс watchdog также очищает RAM при обычном старте.
bool alarmLatched = false;

struct OutputState {
#if !LIN_DRIVER
  bool key1;
  bool key2;
#else
  bool key;
#endif
};

#if !LIN_DRIVER

// ============================================================
// Варианты поведения PA6 / PA7
// ============================================================
//
// MOD_SINGLE_HIGH:
//   NORMAL       PA6=0, PA7=0
//   ALARM        PA6=0, PA7=1
//
// MOD_SINGLE_LOW:
//   NORMAL       PA6=0, PA7=0
//   ALARM        PA6=1, PA7=0
//
// MOD_DUAL_HIGH:
//   NORMAL       PA6=0, PA7=0
//   ALARM        PA6=1, PA7=1
//
// MOD_COMPLEMENTARY:
//   NORMAL       PA6=1, PA7=0
//   ALARM        PA6=0, PA7=1
//
// MOD_ACTIVE_LOW:
//   NORMAL       PA6=1, PA7=1
//   ALARM        PA6=0, PA7=0
//
// Во время WARMUP и состояний неисправности датчика PA6/PA7 выключены.

enum SensorModification {
  MOD_SINGLE_HIGH,      // ключ OUT1 с активным высоким уровнем, высокий уровень на выходе при ALARM, OUT2 неиспользуется, выключен
  MOD_SINGLE_LOW,       // ключ OUT1 с активным низким уровнем, низкий уровень на выходе при ALARM, OUT2 неиспользуется, выключен
  MOD_DUAL_HIGH,        // ключи OUT1 и OUT2 с активным высоким уровнем, высокий уровень на обоих выходах при ALARM
  MOD_COMPLEMENTARY,    // ключи OUT1 и OUT2 с активным высоким уровнем, при ALARM OUT1 включен, OUT2 выключен
  MOD_ACTIVE_LOW        // ключи OUT1 и OUT2 с активным низким уровнем, низкий уровень на обоих выходах при ALARM
};

// ------------ ВЫБЕРИТЕ МОДИФИКАЦИЮ ДАТЧИКА ЗДЕСЬ ------------
const SensorModification SENSOR_MODIFICATION = MOD_SINGLE_LOW;
// -------------------------------------------------------------

const OutputState OUTPUT_SAFE_OFF = { false, false };

OutputState getOutputState(SystemState state)
{
  // if (state == STATE_WARMUP ||
  //     state == STATE_SENSOR_OPEN_OR_GND ||
  //     state == STATE_SENSOR_SHORT_TO_VCC) {
  //   return OUTPUT_SAFE_OFF;
  // }
  
  bool alarm = (state == STATE_ALARM);

  if (state == STATE_WARMUP){
    alarm = false;
  }
  
  if (state == STATE_SENSOR_OPEN_OR_GND ||
      state == STATE_SENSOR_SHORT_TO_VCC) {
    return OUTPUT_SAFE_OFF;
  }

  switch (SENSOR_MODIFICATION) {
    case MOD_SINGLE_HIGH:
      if (alarm) return { true,  false };
      else       return { false, false };

    case MOD_SINGLE_LOW:
      if (alarm) return { false, false  };
      else       return { true, false };

    case MOD_DUAL_HIGH:
      if (alarm) return { true,  true  };
      else       return { false, false };

    case MOD_COMPLEMENTARY:
      if (alarm) return { true,  false };
      else       return { false, true  };

    case MOD_ACTIVE_LOW:
      if (alarm) return { false, false };
      else       return { true,  true  };

    default:
      return OUTPUT_SAFE_OFF;
  }
}

void applyOutputs(SystemState state)
{
  OutputState out = getOutputState(state);

  digitalWrite(KEY1_PIN, out.key1 ? HIGH : LOW);
  digitalWrite(KEY2_PIN, out.key2 ? HIGH : LOW);
}

#else

// ============================================================
// Варианты поведения PA7
// ============================================================
//
// MOD_SINGLE_HIGH:
//   NORMAL       PA7=0
//   ALARM        PA7=1
//
// MOD_SINGLE_LOW:
//   NORMAL       PA7=0
//   ALARM        PA7=0
//
//

enum SensorModification {
  MOD_SINGLE_HIGH,     // ключ OUT с активным высоким уровнем, высокий уровень на выходе при ALARM
  MOD_SINGLE_LOW       // ключ OUT с активным низким уровнем, низкий уровень на выходе при ALARM
};

// ------------ ВЫБЕРИТЕ МОДИФИКАЦИЮ ДАТЧИКА ЗДЕСЬ ------------
const SensorModification SENSOR_MODIFICATION = MOD_SINGLE_LOW;
// -------------------------------------------------------------

const OutputState OUTPUT_SAFE_OFF = { false };

OutputState getOutputState(SystemState state)
{
  // if (state == STATE_SENSOR_OPEN_OR_GND ||
  //     state == STATE_SENSOR_SHORT_TO_VCC ||
  //     state == STATE_WARMUP) {
  //   return OUTPUT_SAFE_OFF;
  // }

  // return { state == STATE_ALARM };

  bool alarm = (state == STATE_ALARM);

  if (state == STATE_WARMUP){
    alarm = false;
  }
  
  if (state == STATE_SENSOR_OPEN_OR_GND ||
      state == STATE_SENSOR_SHORT_TO_VCC) {
    return OUTPUT_SAFE_OFF;
  }

  switch (SENSOR_MODIFICATION) {
    case MOD_SINGLE_HIGH:
      if (alarm) return { true };
      else       return { false };

    case MOD_SINGLE_LOW:
      if (alarm) return { false };
      else       return { true };

    default:
      return OUTPUT_SAFE_OFF;
  }
}

void applyOutputs(SystemState state)
{
  OutputState out = getOutputState(state);
  digitalWrite(KEY_PIN, out.key ? HIGH : LOW);
}

#endif

// ============================================================
// LIN slave
// ============================================================

#if LIN_DRIVER

uint8_t linProtectedId(uint8_t id)
{
  id &= 0x3F;

  uint8_t p0 =
    ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
  uint8_t p1 =
    (~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5))) & 0x01;

  return id | (p0 << 6) | (p1 << 7);
}

bool linPidIsValid(uint8_t pid)
{
  return linProtectedId(pid & 0x3F) == pid;
}

uint8_t linChecksum(uint8_t pid, const uint8_t *data, uint8_t len)
{
  uint16_t sum = pid;

  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum > 0xFF) {
      sum = (sum & 0xFF) + 1;
    }
  }

  return (uint8_t)(~sum);
}

uint8_t linStateCode(SystemState state)
{
  return (uint8_t)state;
}

bool outputIsActive()
{
  OutputState out = getOutputState(systemState);
  return out.key;
}

bool faultIsActive()
{
  return systemState == STATE_SENSOR_OPEN_OR_GND ||
         systemState == STATE_SENSOR_SHORT_TO_VCC;
}

void linSendStatus(uint8_t pid)
{
  uint8_t flags = 0;

  if (alarmLatched) {
    flags |= 0x01;
  }
  if (outputIsActive()) {
    flags |= 0x02;
  }
  if (faultIsActive()) {
    flags |= 0x04;
  }
  if (systemState == STATE_WARMUP) {
    flags |= 0x08;
  }

  uint8_t data[LIN_STATUS_DATA_LEN] = {
    linStateCode(systemState),
    flags,
    (uint8_t)(filteredAdc & 0xFF),
    (uint8_t)(filteredAdc >> 8)
  };

  Serial.write(data, LIN_STATUS_DATA_LEN);
  Serial.write(linChecksum(pid, data, LIN_STATUS_DATA_LEN));
  Serial.flush();
}

void linProcessByte(uint8_t b)
{
  if (millis() - lastLinByteMs > LIN_HEADER_TIMEOUT_MS) {
    linRxState = LIN_WAIT_SYNC;
  }
  lastLinByteMs = millis();

  if (b == 0x00) {
    linRxState = LIN_WAIT_SYNC;
    return;
  }

  switch (linRxState) {
    case LIN_WAIT_SYNC:
      if (b == 0x55) {
        linRxState = LIN_WAIT_PID;
      }
      break;

    case LIN_WAIT_PID:
      linRxState = LIN_WAIT_SYNC;

      if (!linPidIsValid(b)) {
        return;
      }

      if (b == linProtectedId(LIN_STATUS_ID)) {
        linSendStatus(b);
      }
      break;
  }
}

void linService()
{
  while (Serial.available() > 0) {
    int b = Serial.read();

    if (b >= 0) {
      linProcessByte((uint8_t)b);
    }
  }
}

void linBegin()
{
  pinMode(LIN_TX_PIN, OUTPUT);
  pinMode(LIN_RX_PIN, INPUT_PULLUP);

  Serial.swap(1);
  Serial.begin(LIN_BAUD, SERIAL_8N1);
}

#endif

void serviceDelay(uint16_t delayMs)
{
#if LIN_DRIVER
  uint32_t startMs = millis();

  while (millis() - startMs < delayMs) {
    linService();
    delay(1);
  }
#else
  delay(delayMs);
#endif
}

// ============================================================
// ADC
// ============================================================

uint16_t readAdcAverage()
{
  uint32_t sum = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(SENSOR_PIN);
    serviceDelay(2);
  }

  return sum / ADC_SAMPLES;
}

// ============================================================
// Логика газового порога
// ============================================================

bool isAlarmOnLevel(uint16_t adc)
{
  if (SENSOR_SIGNAL_INCREASES_WITH_GAS) {
    return adc >= activeGasAlarmOnAdc;
  } else {
    return adc <= activeGasAlarmOnAdc;
  }
}

bool isAlarmOffLevel(uint16_t adc)
{
  if (SENSOR_SIGNAL_INCREASES_WITH_GAS) {
    return adc <= activeGasAlarmOffAdc;
  } else {
    return adc >= activeGasAlarmOffAdc;
  }
}

// ============================================================
// Логика уровней неисправности датчика
// ============================================================

bool isSensorOpenOrGndOn(uint16_t adc)
{
  return adc <= ADC_SENSOR_OPEN_OR_GND_ON;
}

bool isSensorOpenOrGndOff(uint16_t adc)
{
  return adc >= ADC_SENSOR_OPEN_OR_GND_OFF;
}

bool isSensorShortToVccOn(uint16_t adc)
{
  return adc >= ADC_SENSOR_SHORT_TO_VCC_ON;
}

bool isSensorShortToVccOff(uint16_t adc)
{
  return adc <= ADC_SENSOR_SHORT_TO_VCC_OFF;
}

// ============================================================
// Автокалибровка чистого воздуха
// ============================================================

#if AUTOCALIBR

uint16_t adcAbsDiff(uint16_t a, uint16_t b)
{
  return (a >= b) ? (a - b) : (b - a);
}

uint16_t clampAdc(int16_t adc)
{
  if (adc < 0) {
    return 0;
  }

  if (adc > ADC_MAX_VALUE) {
    return ADC_MAX_VALUE;
  }

  return (uint16_t)adc;
}

uint8_t autocalibrChecksum(uint16_t cleanAirAdc)
{
  return (uint8_t)(
    0xA5 ^
    (AUTOCALIBR_EEPROM_SIGNATURE & 0xFF) ^
    (AUTOCALIBR_EEPROM_SIGNATURE >> 8) ^
    (cleanAirAdc & 0xFF) ^
    (cleanAirAdc >> 8)
  );
}

uint16_t eepromReadUint16(uint8_t addr)
{
  return (uint16_t)EEPROM.read(addr) |
         ((uint16_t)EEPROM.read(addr + 1) << 8);
}

void eepromUpdateUint16(uint8_t addr, uint16_t value)
{
  EEPROM.update(addr, (uint8_t)(value & 0xFF));
  EEPROM.update(addr + 1, (uint8_t)(value >> 8));
}

bool loadCalibration(uint16_t &cleanAirAdc)
{
  uint16_t signature = eepromReadUint16(AUTOCALIBR_EEPROM_ADDR);
  uint16_t storedAdc = eepromReadUint16(AUTOCALIBR_EEPROM_ADDR + 2);
  uint8_t storedChecksum = EEPROM.read(AUTOCALIBR_EEPROM_ADDR + 4);

  if (signature != AUTOCALIBR_EEPROM_SIGNATURE ||
      storedAdc > ADC_MAX_VALUE ||
      storedChecksum != autocalibrChecksum(storedAdc)) {
    return false;
  }

  cleanAirAdc = storedAdc;
  return true;
}

void saveCalibration(uint16_t cleanAirAdc)
{
  eepromUpdateUint16(AUTOCALIBR_EEPROM_ADDR, AUTOCALIBR_EEPROM_SIGNATURE);
  eepromUpdateUint16(AUTOCALIBR_EEPROM_ADDR + 2, cleanAirAdc);
  EEPROM.update(AUTOCALIBR_EEPROM_ADDR + 4, autocalibrChecksum(cleanAirAdc));
}

void applyCalibration(uint16_t cleanAirAdc)
{
  int16_t alarmOnOffset =
    (int16_t)GAS_ALARM_ON_ADC - (int16_t)GAS_CLEAN_AIR_FACTORY_ADC;
  int16_t alarmOffOffset =
    (int16_t)GAS_ALARM_OFF_ADC - (int16_t)GAS_CLEAN_AIR_FACTORY_ADC;

  activeGasAlarmOnAdc = clampAdc((int16_t)cleanAirAdc + alarmOnOffset);
  activeGasAlarmOffAdc = clampAdc((int16_t)cleanAirAdc + alarmOffOffset);
}

void runInitialCalibration(uint16_t warmupAdc)
{
  if (isSensorOpenOrGndOn(warmupAdc) || isSensorShortToVccOn(warmupAdc)) {
    return;
  }

  uint16_t storedCleanAirAdc = 0;
  bool hasStoredCalibration = loadCalibration(storedCleanAirAdc);
  uint16_t selectedCleanAirAdc = warmupAdc;
  bool writeCalibration = false;

  if (hasStoredCalibration) {
    if (adcAbsDiff(warmupAdc, storedCleanAirAdc) <= AUTOCALIBR_MAX_DELTA_ADC) {
      selectedCleanAirAdc = warmupAdc;
      writeCalibration = (warmupAdc != storedCleanAirAdc);
    } else {
      selectedCleanAirAdc = storedCleanAirAdc;
    }
  } else {
    writeCalibration = true;
  }

  applyCalibration(selectedCleanAirAdc);

  if (writeCalibration) {
    saveCalibration(selectedCleanAirAdc);
  }

#if DEBUG
  debugPrint("Calibration clean_air=");
  debugPrintUint(selectedCleanAirAdc);
  debugPrint("  gas_on=");
  debugPrintUint(activeGasAlarmOnAdc);
  debugPrint(writeCalibration ? "  saved" : "  kept");
  debugPrint("\r\n");
#endif
}
#endif

// ============================================================
// Индикация светодиодом
// ============================================================
//
// В базовой версии светодиод на PA1. В LIN-версии светодиод на PA3,
// но при DEBUG он отключается, потому что PA3 используется как отладочный TX.
//
// WARMUP              - короткая вспышка раз в секунду
// NORMAL              - короткая вспышка раз в 2 секунды
// ALARM               - частое мигание, защелкнуто до отключения питания
// SENSOR_OPEN_OR_GND  - две вспышки, пауза
// SENSOR_SHORT_TO_VCC - три вспышки, пауза

void ledSet(bool on)
{
#if LED_ENABLED
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void updateLed(SystemState state)
{
  uint32_t t = millis();

  switch (state) {
    case STATE_WARMUP:
      ledSet((t % 1000UL) < 100UL);
      break;

    case STATE_NORMAL:
      ledSet((t % 2000UL) < 80UL);
      break;

    case STATE_ALARM:
      ledSet((t % 250UL) < 125UL);
      break;

    case STATE_SENSOR_OPEN_OR_GND: {
      uint32_t p = t % 2000UL;
      bool on =
        (p < 120UL) ||
        (p >= 300UL && p < 420UL);
      ledSet(on);
      break;
    }

    case STATE_SENSOR_SHORT_TO_VCC: {
      uint32_t p = t % 2000UL;
      bool on =
        (p < 120UL) ||
        (p >= 300UL && p < 420UL) ||
        (p >= 600UL && p < 720UL);
      ledSet(on);
      break;
    }

    default:
      ledSet(false);
      break;
  }
}

// ============================================================
// Вспомогательные функции состояния
// ============================================================

void setState(SystemState newState)
{
  if (systemState != newState) {
    systemState = newState;
    applyOutputs(systemState);
  }
}

const char *stateToText(SystemState state)
{
  switch (state) {
    case STATE_WARMUP:              return "WARMUP";
    case STATE_NORMAL:              return "NORMAL";
    case STATE_ALARM:               return "ALARM";
    case STATE_SENSOR_OPEN_OR_GND:  return "SENSOR_OPEN_OR_GND";
    case STATE_SENSOR_SHORT_TO_VCC: return "SENSOR_SHORT_TO_VCC";
    default:                        return "UNKNOWN";
  }
}

// ============================================================
// Глобальные переменные
// ============================================================

uint32_t alarmStartMs = 0;
uint32_t faultStartMs = 0;
uint32_t faultReleaseStartMs = 0;
uint32_t lastDebugMs = 0;

// ============================================================
// Отладочный статус
// ============================================================
#if DEBUG
void debugStatus(uint16_t adcRaw)
{
  debugPrint("state=");
  debugPrint(stateToText(systemState));

  debugPrint("  adc=");
  debugPrintUint(adcRaw);

  debugPrint("  filt=");
  debugPrintUint(filteredAdc);

  debugPrint("  gas_on=");
  debugPrintUint(activeGasAlarmOnAdc);

  debugPrint("  open_gnd_on=");
  debugPrintUint(ADC_SENSOR_OPEN_OR_GND_ON);

  debugPrint("  short_vcc_on=");
  debugPrintUint(ADC_SENSOR_SHORT_TO_VCC_ON);

  debugPrint("  out=");
  OutputState out = getOutputState(systemState);
#if !LIN_DRIVER
  debugPrint(out.key1 ? "1" : "0");
  debugPrint(",");
  debugPrint(out.key2 ? "1" : "0");
#else
  debugPrint(out.key ? "1" : "0");
#endif

  debugPrint("  latched=");
  debugPrint(alarmLatched ? "1" : "0");

  debugPrint("\r\n");
}

void debugAutocalibrSettings()
{
  debugPrint("autocalibr=");
#if AUTOCALIBR
  debugPrint("1");

  debugPrint("  factory_clean_air=");
  debugPrintUint(GAS_CLEAN_AIR_FACTORY_ADC);

  debugPrint("  max_delta=");
  debugPrintUint(AUTOCALIBR_MAX_DELTA_ADC);

  debugPrint("  gas_on=");
  debugPrintUint(activeGasAlarmOnAdc);

  debugPrint("  gas_off=");
  debugPrintUint(activeGasAlarmOffAdc);
#else
  debugPrint("0");

  debugPrint("  factory_clean_air=");
  debugPrintUint(GAS_CLEAN_AIR_FACTORY_ADC);

  debugPrint("  gas_on=");
  debugPrintUint(GAS_ALARM_ON_ADC);

  debugPrint("  gas_off=");
  debugPrintUint(GAS_ALARM_OFF_ADC);
#endif

  debugPrint("\r\n");
}
#endif

// ============================================================
// Обработка неисправностей датчика
// ============================================================

bool processFaults(uint16_t adc)
{
  uint32_t now = millis();

  // Уже в состоянии: датчик оборван или линия датчика подключена к GND
  if (systemState == STATE_SENSOR_OPEN_OR_GND) {
    if (isSensorOpenOrGndOff(adc)) {
      if (faultReleaseStartMs == 0) {
        faultReleaseStartMs = now;
      }

      if (now - faultReleaseStartMs >= FAULT_RELEASE_MS) {
        faultReleaseStartMs = 0;
        faultStartMs = 0;
        setState(STATE_NORMAL);
        return false;
      }
    } else {
      faultReleaseStartMs = 0;
    }

    return true;
  }

  // Уже в состоянии: выход датчика замкнут на питание
  if (systemState == STATE_SENSOR_SHORT_TO_VCC) {
    if (isSensorShortToVccOff(adc)) {
      if (faultReleaseStartMs == 0) {
        faultReleaseStartMs = now;
      }

      if (now - faultReleaseStartMs >= FAULT_RELEASE_MS) {
        faultReleaseStartMs = 0;
        faultStartMs = 0;
        setState(STATE_NORMAL);
        return false;
      }
    } else {
      faultReleaseStartMs = 0;
    }

    return true;
  }

  // Новая неисправность: SENSOR_PIN около 0 В
  if (isSensorOpenOrGndOn(adc)) {
    if (faultStartMs == 0) {
      faultStartMs = now;
    }

    if (now - faultStartMs >= FAULT_CONFIRM_MS) {
      faultStartMs = 0;
      faultReleaseStartMs = 0;
      alarmStartMs = 0;
      setState(STATE_SENSOR_OPEN_OR_GND);
      return true;
    }

    return false;
  }

  // Новая неисправность: SENSOR_PIN около +5 В
  if (isSensorShortToVccOn(adc)) {
    if (faultStartMs == 0) {
      faultStartMs = now;
    }

    if (now - faultStartMs >= FAULT_CONFIRM_MS) {
      faultStartMs = 0;
      faultReleaseStartMs = 0;
      alarmStartMs = 0;
      setState(STATE_SENSOR_SHORT_TO_VCC);
      return true;
    }

    return false;
  }

  faultStartMs = 0;
  faultReleaseStartMs = 0;

  return false;
}

// ============================================================
// Настройка
// ============================================================

void setup()
{
#if LED_ENABLED
  pinMode(LED_PIN, OUTPUT);
#endif

#if !LIN_DRIVER
  pinMode(KEY1_PIN, OUTPUT);
  pinMode(KEY2_PIN, OUTPUT);
#else
  pinMode(KEY_PIN, OUTPUT);
  linBegin();
#endif

  // На плате установлена внешняя подтяжка 4.7 кОм к GND.
  // Внутренняя подтяжка не должна быть включена.
  pinMode(SENSOR_PIN, INPUT);

#if DEBUG
  pinMode(DEBUG_TX_PIN, OUTPUT);
  digitalWrite(DEBUG_TX_PIN, HIGH); // уровень покоя UART
#endif

  setState(STATE_WARMUP);
  applyOutputs(STATE_WARMUP);

  #if DEBUG
  debugPrintln("");
  debugPrintln("ATtiny412 gas sensor controller start");
  debugPrintln("ALARM is latched until power cycle");
  #endif

  uint32_t warmupStart = millis();

  while (millis() - warmupStart < SENSOR_WARMUP_MS) {
    updateLed(STATE_WARMUP);
    applyOutputs(STATE_WARMUP);
    serviceDelay(20);
  }

  filteredAdc = readAdcAverage();

#if AUTOCALIBR
  runInitialCalibration(filteredAdc);
#endif

  #if DEBUG
  debugPrintln("Warmup done");
  #endif
}

// ============================================================
// Основной цикл
// ============================================================

void loop()
{
  uint16_t adcRaw = readAdcAverage();

  filteredAdc =
    ((uint32_t)filteredAdc * (FILTER_DIV - 1) + adcRaw) / FILTER_DIV;

  // ----------------------------------------------------------
  // Если ALARM защелкнут, он не может вернуться в NORMAL.
  // Сброс только снятием питания с контроллера.
  // Остальные состояния могут вернуться в NORMAL.
  // ----------------------------------------------------------
  if (alarmLatched) {
    setState(STATE_ALARM);
  } else {
    // Сначала обработать диагностические состояния напряжения датчика.
    // Состояния неисправности датчика восстанавливаемые.
    bool faultActive = processFaults(filteredAdc);

    if (!faultActive &&
        systemState != STATE_SENSOR_OPEN_OR_GND &&
        systemState != STATE_SENSOR_SHORT_TO_VCC) {

      // Ждать, пока газовый порог будет превышен непрерывно.
      if (isAlarmOnLevel(filteredAdc)) {
        if (alarmStartMs == 0) {
          alarmStartMs = millis();
        }

        if (millis() - alarmStartMs >= ALARM_CONFIRM_MS) {
          alarmLatched = true;
          setState(STATE_ALARM);
        }
      } else {
        alarmStartMs = 0;
        setState(STATE_NORMAL);
      }
    }
  }

  applyOutputs(systemState);
  updateLed(systemState);

  #if DEBUG
  if (millis() - lastDebugMs >= DEBUG_PERIOD_MS) {
    lastDebugMs = millis();
    debugStatus(adcRaw);
    debugAutocalibrSettings();
  }
  #endif

  serviceDelay(50);
}
