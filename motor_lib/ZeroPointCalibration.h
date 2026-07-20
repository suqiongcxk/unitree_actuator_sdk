#ifndef __ZERO_POINT_CALIBRATION_H
#define __ZERO_POINT_CALIBRATION_H

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>

struct  LEG_UART_SET
{
    int GPIO_PIN;
    const char* SERIAL_PORT;
};


struct  SINGLE_LEG_MOtor_SET
{
    int HIP_Motor;
    int thigh_Motor;
    int lower_leg_Motor;
    LEG_UART_SET Leg_UART;
};

extern LEG_UART_SET Leg1_Uart;
extern LEG_UART_SET Leg2_Uart;
extern LEG_UART_SET Leg3_Uart;
extern  LEG_UART_SET Leg4_Uart;
extern  SINGLE_LEG_MOtor_SET Leg1_Motor;
extern  SINGLE_LEG_MOtor_SET Leg2_Motor;
extern  SINGLE_LEG_MOtor_SET Leg3_Motor;
extern  SINGLE_LEG_MOtor_SET Leg4_Motor;

void ZeroPointCalibration(void );
void LEG_MOTOR_INIT(void);
#endif  // __ZERO_POINT_CALIBRATION_H