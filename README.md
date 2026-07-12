# ATtiny412 gas sensor controller

Pin assignment:

- PA0: UPDI
- PA1: board status LED
- PA2: ADC sensor input with external 4.7 kOhm pull-down to GND
- PA3: debug TX, software UART 1200 baud
- PA6: power switch output 1
- PA7: power switch output 2

Main behavior:

- no calibration after warmup;
- sensor open or line to GND is detected by ADC near 0 V;
- sensor output shorted to supply is detected by ADC near +5 V;
- ALARM state is latched until controller power cycle;
- other states are recoverable.

Change `SENSOR_MODIFICATION` in `src/main.cpp` to select PA6/PA7 behavior.
