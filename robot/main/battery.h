/* Battery charge from the voltage divider on BATTERY_ADC_GPIO, read on core 0 */
#pragma once

/* Set up the ADC and start the task that samples it. If the pin can't be
   read, this logs why and there is never a reading. */
void battery_start(void);

/* Filtered voltage at the pin (mV) and the charge from it (0-100 %),
   both -1 if there is no reading. Either pointer may be NULL. */
void battery_get(int *pin_mv, int *percent);
