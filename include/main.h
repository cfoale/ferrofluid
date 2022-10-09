#ifndef MAIN_H
#define MAIN_H

float ChargeCapacitor(const float vmax, const unsigned int interval);
float* Measure(const bool coil1_active, const unsigned int initial_delay,
               const unsigned int interval, const unsigned int max_time);

#endif // MAIN_H
