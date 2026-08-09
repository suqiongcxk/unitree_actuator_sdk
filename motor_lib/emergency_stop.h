#ifndef __EMERGENCY_STOP_H
#define __EMERGENCY_STOP_H

// 进程内统一急停标志：信号处理器和键盘线程只负责置位。
void requestEmergencyStop() noexcept;
bool isEmergencyStopRequested() noexcept;
void resetEmergencyStop() noexcept;

#endif  // __EMERGENCY_STOP_H
