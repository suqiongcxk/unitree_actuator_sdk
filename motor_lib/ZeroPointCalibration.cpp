#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include "motor_controller.h"
#include "ZeroPointCalibration.h"
// ── 硬件配置库 ──────────────────────────────────────────
constexpr int GPIO_PIN[4] = {63, 39, 35, 133};
constexpr const char *SERIAL_PORT[4] = {"/dev/ttyS4", "/dev/ttyS6", "/dev/ttyS7", "/dev/ttyS0"};
constexpr unsigned short MOTOR_ID[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
float ZERO_Position[12] ={0};  //校准后零角度位置
LEG_UART_SET Leg1_Uart;
LEG_UART_SET Leg2_Uart;
LEG_UART_SET Leg3_Uart;
LEG_UART_SET Leg4_Uart;
SINGLE_LEG_MOtor_SET Leg1_Motor;
SINGLE_LEG_MOtor_SET Leg2_Motor;
SINGLE_LEG_MOtor_SET Leg3_Motor;
SINGLE_LEG_MOtor_SET Leg4_Motor;
void LEG_UART_INIT(void)
{
    Leg1_Uart = {133, "/dev/ttyS0"};
    Leg2_Uart = {39,  "/dev/ttyS6"};
    Leg3_Uart = {35,  "/dev/ttyS7"};
    Leg4_Uart = {63,  "/dev/ttyS4"};
};

void LEG_MOTOR_INIT(void)
{
    LEG_UART_INIT();
     Leg1_Motor = {
        0, // hip
        4, // thigh
        8, // lower_leg
        Leg1_Uart};
     Leg2_Motor = {
        1, // hip
        5, // thigh
        9, // lower_leg
        Leg2_Uart};
     Leg3_Motor = {
        2, // hip
        6, // thigh
        10, // lower_leg
        Leg3_Uart};
     Leg4_Motor = {
        3, // hip
        7, // thigh
        11, // lower_leg
        Leg4_Uart};
};

void ZeroPointCalibration(void )
{
    MotorState State_Text;
    // 三个电机共享同一条 RS-485 总线，用 MotorBus 避免 GPIO 冲突
    MotorBus bus(Leg1_Motor.Leg_UART.GPIO_PIN, Leg1_Motor.Leg_UART.SERIAL_PORT);
    bus.addMotor(Leg1_Motor.HIP_Motor);
    bus.addMotor(Leg1_Motor.thigh_Motor);
    bus.addMotor(Leg1_Motor.lower_leg_Motor);

    // 小腿
    bus.setVelocity(Leg1_Motor.lower_leg_Motor, -1, 0.06);
    bus.sendRecv();
    sleep(3);
    bus.setVelocity(Leg1_Motor.lower_leg_Motor, -1, 0.06);
    bus.sendRecv();
    State_Text = bus.getState(Leg1_Motor.lower_leg_Motor);
    ZERO_Position[Leg1_Motor.lower_leg_Motor] = State_Text.q;

    // 大腿
    bus.setVelocity(Leg1_Motor.thigh_Motor, 1.0, 0.08);
    bus.sendRecv();
    sleep(4);
    bus.setVelocity(Leg1_Motor.thigh_Motor, 1.0, 0.08 );
    bus.sendRecv();
    State_Text = bus.getState(Leg1_Motor.thigh_Motor);
    ZERO_Position[Leg1_Motor.thigh_Motor] = State_Text.q;

    // 髋关节
    bus.setVelocity(Leg1_Motor.HIP_Motor, 1, 0.08);
    bus.sendRecv();
    sleep(3);
    bus.setVelocity(Leg1_Motor.HIP_Motor, 1, 0.08);
    bus.sendRecv();
    State_Text = bus.getState(Leg1_Motor.HIP_Motor);
    ZERO_Position[Leg1_Motor.HIP_Motor] = State_Text.q;


    std::cout << "hip:"        << ZERO_Position[Leg1_Motor.HIP_Motor]       << std::endl;
    std::cout << "thigh:"      << ZERO_Position[Leg1_Motor.thigh_Motor]     << std::endl;
    std::cout << "lower_leg:"  << ZERO_Position[Leg1_Motor.lower_leg_Motor] << std::endl;
}