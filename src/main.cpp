#include <Arduino.h>

// ============================================================
// ATtiny412 gas sensor controller
// ============================================================
// PA0 - UPDI, do not use as GPIO
// PA1 - board status LED
// PA2 - ADC sensor input, external pull-down 4.7 kOhm to GND
// PA3 - debug TX, software UART 1200 baud
// PA6 - power switch output 1
// PA7 - power switch output 2
//
// Logic:
// - sensor voltage is checked by fixed ADC thresholds
// - ADC near 0 V: sensor open or sensor line connected to GND
// - ADC near +5 V: sensor output shorted to VCC
// - ALARM state is latched in RAM and can be reset only by power cycling
// - other states can return to NORMAL when ADC returns to valid range
// ============================================================

#define LED_PIN       PIN_PA1
#define SENSOR_PIN    PIN_PA2
#define DEBUG_TX_PIN  PIN_PA3
#define KEY1_PIN      PIN_PA6
#define KEY2_PIN      PIN_PA7

// ============================================================
// Debug software UART on PA3
// ============================================================

#define DEBUG_BAUD 1200UL
#define BIT_TIME_US (1000000UL / DEBUG_BAUD)

void debugTxWriteBit(bool bitValue)
{
  digitalWrite(DEBUG_TX_PIN, bitValue ? HIGH : LOW);
  delayMicroseconds(BIT_TIME_US);
}

void debugTxWriteByte(uint8_t b)
{
  noInterrupts();

  // Start bit
  debugTxWriteBit(false);

  // Data bits, LSB first
  for (uint8_t i = 0; i < 8; i++) {
    debugTxWriteBit(b & 0x01);
    b >>= 1;
  }

  // Stop bit
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

// ============================================================
// Main settings
// ============================================================

// ADC is 10-bit in Arduino analogRead(): 0...1023
const uint16_t ADC_MAX_VALUE = 1023;

// Sensor warmup time after power-up.
// During warmup PA6/PA7 stay in safe state.
const uint32_t SENSOR_WARMUP_MS = 30000UL;

// Debug output period
const uint32_t DEBUG_PERIOD_MS = 1000UL;

// Gas alarm confirmation time.
// ALARM is latched only if gas threshold is exceeded continuously
// for this time.
const uint32_t ALARM_CONFIRM_MS = 3000UL;

// Sensor fault confirmation time by min/max voltage
const uint32_t FAULT_CONFIRM_MS = 1000UL;

// Sensor fault release time.
// FAULT states can return to NORMAL when ADC returns to valid range.
const uint32_t FAULT_RELEASE_MS = 1000UL;

// ADC averaging
const uint8_t ADC_SAMPLES = 16;

// ADC low-pass filter coefficient.
// 8 means: filtered = filtered * 7/8 + raw * 1/8
const uint8_t FILTER_DIV = 8;

// ============================================================
// Gas thresholds
// ============================================================

// true  - ADC signal increases when gas is present
// false - ADC signal decreases when gas is present
const bool SENSOR_SIGNAL_INCREASES_WITH_GAS = true;

// Gas alarm ON threshold
const uint16_t GAS_ALARM_ON_ADC = 650;

// Gas alarm OFF threshold is not used to reset ALARM anymore,
// because ALARM is latched until power cycle.
// It is kept only for possible future non-latched variants.
const uint16_t GAS_ALARM_OFF_ADC = 580;

// ============================================================
// Sensor voltage diagnostic thresholds
// ============================================================
//
// PA2 has external 4.7 kOhm pull-down to GND.
//
// ADC <= ADC_SENSOR_OPEN_OR_GND_ON:
//   sensor open or sensor line connected to GND.
//   PA2 voltage is near 0 V.
//
// ADC >= ADC_SENSOR_SHORT_TO_VCC_ON:
//   sensor output shorted to supply.
//   PA2 voltage is near +5 V.
//
// OFF thresholds provide hysteresis for returning from fault states.

const uint16_t ADC_SENSOR_OPEN_OR_GND_ON  = 20;
const uint16_t ADC_SENSOR_OPEN_OR_GND_OFF = 50;

const uint16_t ADC_SENSOR_SHORT_TO_VCC_ON  = 1000;
const uint16_t ADC_SENSOR_SHORT_TO_VCC_OFF = 970;

// Approximate values at VCC = 5 V:
// ADC 20   ~= 0.10 V
// ADC 50   ~= 0.24 V
// ADC 970  ~= 4.74 V
// ADC 1000 ~= 4.89 V

// ============================================================
// System states
// ============================================================

enum SystemState {
  STATE_WARMUP,
  STATE_NORMAL,
  STATE_ALARM,
  STATE_SENSOR_OPEN_OR_GND,   // sensor open or sensor line connected to GND
  STATE_SENSOR_SHORT_TO_VCC   // sensor output shorted to supply
};

SystemState systemState = STATE_WARMUP;

// ============================================================
// PA6 / PA7 behavior variants
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
// During WARMUP and sensor fault states PA6/PA7 are switched off.

enum SensorModification {
  MOD_SINGLE_HIGH,
  MOD_SINGLE_LOW,
  MOD_DUAL_HIGH,
  MOD_COMPLEMENTARY,
  MOD_ACTIVE_LOW
};

// ------------ SELECT SENSOR MODIFICATION HERE ---------------
const SensorModification SENSOR_MODIFICATION = MOD_SINGLE_HIGH;
// -------------------------------------------------------------

struct OutputState {
  bool key1;
  bool key2;
};

const OutputState OUTPUT_SAFE_OFF = { false, false };

OutputState getOutputState(SystemState state)
{
  if (state == STATE_WARMUP ||
      state == STATE_SENSOR_OPEN_OR_GND ||
      state == STATE_SENSOR_SHORT_TO_VCC) {
    return OUTPUT_SAFE_OFF;
  }

  bool alarm = (state == STATE_ALARM);

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
// Gas threshold logic
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
// Sensor fault level logic
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
// LED indication
// ============================================================
//
// PA1 works the same for all sensor modifications.
//
// WARMUP              - short blink once per second
// NORMAL              - short blink once per 2 seconds
// ALARM               - fast blinking, latched until power cycle
// SENSOR_OPEN_OR_GND  - two flashes, pause
// SENSOR_SHORT_TO_VCC - three flashes, pause

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
// State helpers
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
// Global variables
// ============================================================

uint16_t filteredAdc = 0;

// ALARM latch. Stored in RAM, so it resets after power cycling.
// Note: hardware reset or watchdog reset also clears RAM on normal startup.
bool alarmLatched = false;

uint32_t alarmStartMs = 0;
uint32_t faultStartMs = 0;
uint32_t faultReleaseStartMs = 0;
uint32_t lastDebugMs = 0;

// ============================================================
// Debug status
// ============================================================

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

// ============================================================
// Sensor fault processing
// ============================================================

bool processFaults(uint16_t adc)
{
  uint32_t now = millis();

  // Already in state: sensor open or sensor line connected to GND
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

  // Already in state: sensor output shorted to supply
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

  // New fault: PA2 near 0 V
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

  // New fault: PA2 near +5 V
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
// Setup
// ============================================================

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(KEY1_PIN, OUTPUT);
  pinMode(KEY2_PIN, OUTPUT);

  // External 4.7 kOhm pull-down to GND is installed on PCB.
  // Internal pull-up must not be enabled.
  pinMode(SENSOR_PIN, INPUT);

  pinMode(DEBUG_TX_PIN, OUTPUT);
  digitalWrite(DEBUG_TX_PIN, HIGH); // UART idle level

  setState(STATE_WARMUP);
  applyOutputs(STATE_WARMUP);

  debugPrintln("");
  debugPrintln("ATtiny412 gas sensor controller start");
  debugPrintln("ALARM is latched until power cycle");

  uint32_t warmupStart = millis();

  while (millis() - warmupStart < SENSOR_WARMUP_MS) {
    updateLed(STATE_WARMUP);
    applyOutputs(STATE_WARMUP);
    delay(20);
  }

  filteredAdc = readAdcAverage();

filteredAdc = readAdcAverage();

  debugPrintln("Warmup done");
}

// ============================================================
// Loop
// ============================================================

void loop()
{
  uint16_t adcRaw = readAdcAverage();

  filteredAdc =
    ((uint32_t)filteredAdc * (FILTER_DIV - 1) + adcRaw) / FILTER_DIV;

  // ----------------------------------------------------------
  // If ALARM has been latched, it cannot return to NORMAL.
  // Reset only by removing controller power.
  // Other states can return to NORMAL.
  // ----------------------------------------------------------
  if (alarmLatched) {
    setState(STATE_ALARM);
  } else {
    // First process sensor voltage diagnostic states.
    // Sensor fault states are recoverable.
    bool faultActive = processFaults(filteredAdc);

    if (!faultActive &&
        systemState != STATE_SENSOR_OPEN_OR_GND &&
        systemState != STATE_SENSOR_SHORT_TO_VCC) {

      // Wait until gas threshold is exceeded continuously.
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

  if (millis() - lastDebugMs >= DEBUG_PERIOD_MS) {
    lastDebugMs = millis();
    debugStatus(adcRaw);
  }

  delay(50);
}
