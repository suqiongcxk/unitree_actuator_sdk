#include <iostream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include "motor_controller.h"
#include "ZeroPointCalibration.h"

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }
// ── 硬件配置库 ──────────────────────────────────────────
constexpr int GPIO_PIN[4] = {63, 39, 35, 133};
constexpr const char *SERIAL_PORT[4] = {"/dev/ttyS4", "/dev/ttyS6", "/dev/ttyS7", "/dev/ttyS0"};
constexpr unsigned short MOTOR_ID[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
float ZERO_Position_MechLimitEnd[12] = {0};   // 逆时针物理限位终点
float ZERO_Position_MechLimitStart[12] = {0}; // 逆时针物理限位起点
float target_angle[12] = {0};
/*读出的弧度制的角度，跟软件限位要求的相比求偏置，
还有就是确认一下角度增大方向跟电机角度增大方向是否相同
limit `[-1.0472, 1.0472] rad` ≈ `[-60°, 60°]    2.0944
limit `[-1.0472, 1.0472] rad` ≈ `[-60°, 60°]
limit `[-1.0472, 1.0472] rad` ≈ `[-60°, 60°]
limit `[-1.0472, 1.0472] rad` ≈ `[-60°, 60°]
limit `[-1.5708, 3.4907] rad` ≈ `[-90°, 200°]
limit `[-1.5708, 3.4907] rad` ≈ `[-90°, 200°]    5.0615
limit `[-0.5236, 4.5379] rad` ≈ `[-30°, 260°]
limit `[-0.5236, 4.5379] rad` ≈ `[-30°, 260°]
limit `[-2.7227, -0.83776] rad` ≈ `[-156°, -48°]
limit `[-2.7227, -0.83776] rad` ≈ `[-156°, -48°]
limit `[-2.7227, -0.83776] rad` ≈ `[-156°, -48°]  1.885
limit `[-2.7227, -0.83776] rad` ≈ `[-156°, -48°]
*/

float URDF_Joint_zero_OFFSET[12] = {0};
// 偏置后的初始站立角度
const float default_joint_pos[12] = {0.1f, -0.1f, 0.1f, -0.1f, 0.8f, 0.8f, 1.0f, 1.0f, -1.5f, -1.5f, -1.5f, -1.5f}; // 从上到下Z字排序
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
   Leg2_Uart = {39, "/dev/ttyS6"};
   Leg3_Uart = {35, "/dev/ttyS7"};
   Leg4_Uart = {63, "/dev/ttyS4"};
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
       2,  // hip
       6,  // thigh
       10, // lower_leg
       Leg3_Uart};
   Leg4_Motor = {
       3,  // hip
       7,  // thigh
       11, // lower_leg
       Leg4_Uart};
};

void ZeroPointCalibration(void)
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
   ZERO_Position_MechLimitStart[Leg1_Motor.lower_leg_Motor] = State_Text.q;

   // 大腿
   bus.setVelocity(Leg1_Motor.thigh_Motor, 1.0, 0.08);
   bus.sendRecv();
   sleep(4);
   bus.setVelocity(Leg1_Motor.thigh_Motor, 1.0, 0.08);
   bus.sendRecv();
   State_Text = bus.getState(Leg1_Motor.thigh_Motor);
   ZERO_Position_MechLimitEnd[Leg1_Motor.thigh_Motor] = State_Text.q;

   // 髋关节
   bus.setVelocity(Leg1_Motor.HIP_Motor, 1, 0.08);
   bus.sendRecv();
   sleep(3);
   bus.setVelocity(Leg1_Motor.HIP_Motor, 1, 0.08);
   bus.sendRecv();
   State_Text = bus.getState(Leg1_Motor.HIP_Motor);
   ZERO_Position_MechLimitEnd[Leg1_Motor.HIP_Motor] = State_Text.q;

   ZERO_Position_MechLimitStart[Leg1_Motor.HIP_Motor] = ZERO_Position_MechLimitEnd[Leg1_Motor.HIP_Motor] - 2 * 1.0472f;
   ZERO_Position_MechLimitStart[Leg1_Motor.thigh_Motor] = ZERO_Position_MechLimitEnd[Leg1_Motor.thigh_Motor] - 5.0615f;
   ZERO_Position_MechLimitEnd[Leg1_Motor.lower_leg_Motor] = ZERO_Position_MechLimitStart[Leg1_Motor.lower_leg_Motor] + 1.885f;

   // 然后将目标在URDF中相对于边界的距离移动到有物理限位的电机中
   /*  调试区
      std::cout << "hipEnd:"        << ZERO_Position_MechLimitEnd[Leg1_Motor.HIP_Motor]       << std::endl;
       std::cout << "thighEnd:"      << ZERO_Position_MechLimitEnd[Leg1_Motor.thigh_Motor]     << std::endl;
       std::cout << "lower_legStart:"  << ZERO_Position_MechLimitStart[Leg1_Motor.lower_leg_Motor] << std::endl;
       std::cout << "hipStart:"        << ZERO_Position_MechLimitStart[Leg1_Motor.HIP_Motor]       << std::endl;
       std::cout << "thighStart:"      << ZERO_Position_MechLimitStart[Leg1_Motor.thigh_Motor]     << std::endl;
       std::cout << "lower_legEnd:"  << ZERO_Position_MechLimitEnd[Leg1_Motor.lower_leg_Motor] << std::endl;
       while(g_running)
       {
         bus.setTorque(Leg1_Motor.HIP_Motor, 0);
         bus.sendRecv();
         State_Text = bus.getState(Leg1_Motor.HIP_Motor);
         std::cout << "hip"<< State_Text.q << std::endl;
         bus.setTorque(Leg1_Motor.thigh_Motor, 0);
         bus.sendRecv();
         State_Text = bus.getState(Leg1_Motor.thigh_Motor);
         std::cout << "thigh"<< State_Text.q << std::endl;
         bus.setTorque(Leg1_Motor.lower_leg_Motor, 0);
         bus.sendRecv();
         State_Text = bus.getState(Leg1_Motor.lower_leg_Motor);
         std::cout << "lower_leg"<< State_Text.q << std::endl;
         sleep(1);
       }
   */
   // 站立测试
   for (int i = 0; i <= 100; i++)
   {
      target_angle[Leg1_Motor.HIP_Motor] = ZERO_Position_MechLimitEnd[Leg1_Motor.HIP_Motor] - (i / 100.0f) * (Joint0_End - default_joint_pos[Leg1_Motor.HIP_Motor]);
      usleep(15000);
      bus.setPosition(Leg1_Motor.HIP_Motor, target_angle[Leg1_Motor.HIP_Motor]  ,0.625, 0.0125);
      bus.sendRecv();
      // std::cout << "TRhip:"        << target_angle[Leg1_Motor.HIP_Motor]       << std::endl;
      // std::cout << "TRthigh:"      << target_angle[Leg1_Motor.thigh_Motor]     << std::endl;
      // std::cout << "TRlower_leg:"<< target_angle[Leg1_Motor.lower_leg_Motor]   << std:end:l;
   }
      for (int i = 0; i <= 100; i++)
   {
      target_angle[Leg1_Motor.thigh_Motor] = ZERO_Position_MechLimitEnd[Leg1_Motor.thigh_Motor] - (i / 100.0f) * (Joint4_End - default_joint_pos[Leg1_Motor.thigh_Motor]);
      target_angle[Leg1_Motor.lower_leg_Motor] = ZERO_Position_MechLimitStart[Leg1_Motor.lower_leg_Motor] + (i / 100.0f) * (default_joint_pos[Leg1_Motor.lower_leg_Motor] - Joint8_Begin);
      usleep(20000);
      bus.setPosition(Leg1_Motor.thigh_Motor, target_angle[Leg1_Motor.thigh_Motor]  ,0.625, 0.0125);
      bus.setPosition(Leg1_Motor.lower_leg_Motor, target_angle[Leg1_Motor.lower_leg_Motor]  ,0.625, 0.0125);
      bus.sendRecv();
   }


}
