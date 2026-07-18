#include <Arduino.h>

// ============================================================
// Контроллер датчика газа на ATtiny412
// ============================================================
// PA0 - UPDI, не использовать как GPIO
// PA1 - светодиод состояния платы
// PA2 - вход АЦП датчика, внешняя подтяжка 4.7 кОм к GND
// PA3 - отладочный TX, программный UART 9600 бод
// PA6 - выход силового ключа 1
// PA7 - выход силового ключа 2
//
// Логика:
// - напряжение датчика проверяется по фиксированным порогам АЦП
// - АЦП около 0 В: датчик оборван или линия датчика подключена к GND
// - АЦП около +5 В: выход датчика замкнут на VCC
// - состояние ALARM защелкивается в RAM и сбрасывается только отключением питания
// - остальные состояния могут вернуться в NORMAL, когда АЦП вернется в допустимый диапазон
// ============================================================

// #define DEBUG
// #define LIN_DRIVER

#ifndef LIN_DRIVER
#define LED_PIN       PIN_PA1
#define SENSOR_PIN    PIN_PA2
#define DEBUG_TX_PIN  PIN_PA3
#define KEY1_PIN      PIN_PA6
#define KEY2_PIN      PIN_PA7
#else
#define LED_PIN       PIN_PA3
#define SENSOR_PIN    PIN_PA6
#define DEBUG_TX_PIN  PIN_PA3 
#define KEY1_PIN      PIN_PA7
#define KEY2_PIN      PIN_PA7
#endif
// ============================================================
// Отладочный программный UART на PA3
// ============================================================


#ifdef DEBUG

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
const uint32_t DEBUG_PERIOD_MS = 1000UL;

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

// Порог включения тревоги по газу
const uint16_t GAS_ALARM_ON_ADC = 650;

// Порог выключения тревоги по газу больше не используется для сброса ALARM,
// потому что ALARM защелкивается до отключения питания.
// Оставлен только для возможных будущих вариантов без защелкивания.
const uint16_t GAS_ALARM_OFF_ADC = 580;

// ============================================================
// Диагностические пороги напряжения датчика
// ============================================================
//
// На PA2 установлена внешняя подтяжка 4.7 кОм к GND.
//
// ADC <= ADC_SENSOR_OPEN_OR_GND_ON:
//   датчик оборван или линия датчика подключена к GND.
//   напряжение на PA2 около 0 В.
//
// ADC >= ADC_SENSOR_SHORT_TO_VCC_ON:
//   выход датчика замкнут на питание.
//   напряжение на PA2 около +5 В.
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
// Состояния системы
// ============================================================

enum SystemState {
  STATE_WARMUP,
  STATE_NORMAL,
  STATE_ALARM,
  STATE_SENSOR_OPEN_OR_GND,   // датчик оборван или линия датчика подключена к GND
  STATE_SENSOR_SHORT_TO_VCC   // выход датчика замкнут на питание
};

SystemState systemState = STATE_WARMUP;

// ============================================================
// Варианты поведения PA6 / PA7
// ============================================================
//
// MOD_SINGLE_HIGH:
//   NORMAL       PA6=0, PA7=0
//   ALARM        PA6=1, PA7=0
//
// MOD_SINGLE_LOW:
//   NORMAL       PA6=0, PA7=0
//   ALARM        PA6=0, PA7=1
//
// MOD_DUAL_HIGH:
//   NORMAL       PA6=0, PA7=0
//   ALARM        PA6=1, PA7=1
//
// MOD_COMPLEMENTARY:
//   NORMAL       PA6=0, PA7=1
//   ALARM        PA6=1, PA7=0
//
// MOD_ACTIVE_LOW:
//   NORMAL       PA6=1, PA7=1
//   ALARM        PA6=0, PA7=0
//
// Во время WARMUP и состояний неисправности датчика PA6/PA7 выключены.

enum SensorModification {
  MOD_SINGLE_HIGH,
  MOD_SINGLE_LOW,
  MOD_DUAL_HIGH,
  MOD_COMPLEMENTARY,
  MOD_ACTIVE_LOW
};

// ------------ ВЫБЕРИТЕ МОДИФИКАЦИЮ ДАТЧИКА ЗДЕСЬ ------------
const SensorModification SENSOR_MODIFICATION = MOD_ACTIVE_LOW;
// -------------------------------------------------------------

struct OutputState {
  bool key1;
  bool key2;
};

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
      if (alarm) return { false, true  };
      else       return { false, false };

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

// ============================================================
// ADC
// ============================================================

uint16_t readAdcAverage()
{
  uint32_t sum = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(SENSOR_PIN);
    delay(2);
  }

  return sum / ADC_SAMPLES;
}

// ============================================================
// Логика газового порога
// ============================================================

bool isAlarmOnLevel(uint16_t adc)
{
  if (SENSOR_SIGNAL_INCREASES_WITH_GAS) {
    return adc >= GAS_ALARM_ON_ADC;
  } else {
    return adc <= GAS_ALARM_ON_ADC;
  }
}

bool isAlarmOffLevel(uint16_t adc)
{
  if (SENSOR_SIGNAL_INCREASES_WITH_GAS) {
    return adc <= GAS_ALARM_OFF_ADC;
  } else {
    return adc >= GAS_ALARM_OFF_ADC;
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
// Индикация светодиодом
// ============================================================
//
// PA1 работает одинаково для всех модификаций датчика.
//
// WARMUP              - короткая вспышка раз в секунду
// NORMAL              - короткая вспышка раз в 2 секунды
// ALARM               - частое мигание, защелкнуто до отключения питания
// SENSOR_OPEN_OR_GND  - две вспышки, пауза
// SENSOR_SHORT_TO_VCC - три вспышки, пауза

void ledSet(bool on)
{
  digitalWrite(LED_PIN, on ? HIGH : LOW);
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

uint16_t filteredAdc = 0;

// Защелка ALARM. Хранится в RAM, поэтому сбрасывается после отключения питания.
// Примечание: аппаратный сброс или сброс watchdog также очищает RAM при обычном старте.
bool alarmLatched = false;

uint32_t alarmStartMs = 0;
uint32_t faultStartMs = 0;
uint32_t faultReleaseStartMs = 0;
uint32_t lastDebugMs = 0;

// ============================================================
// Отладочный статус
// ============================================================
#ifdef DEBUG
void debugStatus(uint16_t adcRaw)
{
  debugPrint("state=");
  debugPrint(stateToText(systemState));

  debugPrint("  adc=");
  debugPrintUint(adcRaw);

  debugPrint("  filt=");
  debugPrintUint(filteredAdc);

  debugPrint("  gas_on=");
  debugPrintUint(GAS_ALARM_ON_ADC);

  debugPrint("  open_gnd_on=");
  debugPrintUint(ADC_SENSOR_OPEN_OR_GND_ON);

  debugPrint("  short_vcc_on=");
  debugPrintUint(ADC_SENSOR_SHORT_TO_VCC_ON);

  debugPrint("  out=");
  OutputState out = getOutputState(systemState);
  debugPrint(out.key1 ? "1" : "0");
  debugPrint(",");
  debugPrint(out.key2 ? "1" : "0");

  debugPrint("  latched=");
  debugPrint(alarmLatched ? "1" : "0");

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

  // Новая неисправность: PA2 около 0 В
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

  // Новая неисправность: PA2 около +5 В
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
  pinMode(LED_PIN, OUTPUT);
  pinMode(KEY1_PIN, OUTPUT);
  pinMode(KEY2_PIN, OUTPUT);

  // На плате установлена внешняя подтяжка 4.7 кОм к GND.
  // Внутренняя подтяжка не должна быть включена.
  pinMode(SENSOR_PIN, INPUT);

  pinMode(DEBUG_TX_PIN, OUTPUT);
  digitalWrite(DEBUG_TX_PIN, HIGH); // уровень покоя UART

  setState(STATE_WARMUP);
  applyOutputs(STATE_WARMUP);

  #ifdef DEBUG
  debugPrintln("");
  debugPrintln("ATtiny412 gas sensor controller start");
  debugPrintln("ALARM is latched until power cycle");
  #endif

  uint32_t warmupStart = millis();

  while (millis() - warmupStart < SENSOR_WARMUP_MS) {
    updateLed(STATE_WARMUP);
    applyOutputs(STATE_WARMUP);
    delay(20);
  }

  filteredAdc = readAdcAverage();

  #ifdef DEBUG
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

  #ifdef DEBUG
  if (millis() - lastDebugMs >= DEBUG_PERIOD_MS) {
    lastDebugMs = millis();
    debugStatus(adcRaw);
  }
  #endif

  delay(50);
}
