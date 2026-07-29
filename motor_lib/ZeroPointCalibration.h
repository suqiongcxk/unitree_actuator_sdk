#ifndef __ZERO_POINT_CALIBRATION_H
#define __ZERO_POINT_CALIBRATION_H

#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>


#define Joint0_Begin   -1.0472
#define Joint0_End      1.0472
#define Joint1_Begin   -1.0472
#define Joint1_End      1.0472

#define Joint3_Begin   -1.0472
#define Joint3_End      1.0472
#define Joint2_Begin   -1.0472
#define Joint2_End      1.0472

#define Joint4_Begin   -1.5708
#define Joint4_End      3.4907
#define Joint5_Begin   -1.5708
#define Joint5_End      3.4907

#define Joint7_Begin   -0.5236
#define Joint7_End      4.5379
#define Joint6_Begin   -0.5236
#define Joint6_End      4.5379

#define Joint8_Begin   -2.7227
#define Joint8_End     -0.83776
#define Joint9_Begin   -2.7227
#define Joint9_End     -0.83776

#define Joint11_Begin  -2.7227
#define Joint11_End   -0.83776
#define Joint10_Begin  -2.7227
#define Joint10_End   -0.83776


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