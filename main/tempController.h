#include "config.h"

#define PID_MAXOUTPUT 1023                      // Maximum control output magnitude (change this if your
                                                // microcontroller has a greater max value)
#define SAMPLING_TIME 1000 / SAMPLING_FREQUENCY // Sampling time (seconds)
#define CONSTANT_Kp 20.0f                       // Proportional gain
#define CONSTANT_Ki 0.0f                        // Integral gain times SAMPLING_TIME
#define CONSTANT_Kd 5.0f                        // Derivative gain divided by SAMPLING_TIME


#define BETA 3950
#define R_REF 10000.0
#define R_NTC 100000.0
#define T0 298.15 // 25 + 273.15


#define e 2.718281828459045235360287471352