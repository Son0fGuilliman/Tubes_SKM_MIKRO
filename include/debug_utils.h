#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <stdint.h>

/* Function Prototypes */
void Print_All_Register_Status(void);
void Print_Control_Header(void);

/**
 * @brief  Prints the current status of the system (Time, Temperature, Setpoint,
 * Fan Speed)
 * @param  time_sec: Current system time in seconds
 * @param  temp: Current temperature
 * @param  setpoint: Current setpoint
 * @param  fan_speed: Current fan speed percentage
 */
void Print_Status(uint32_t time_sec, float temp, float setpoint, int fan_speed);

#endif /* DEBUG_UTILS_H */
