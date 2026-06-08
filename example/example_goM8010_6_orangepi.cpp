#include <unistd.h>
#include <fstream>
#include <iostream>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

// GPIO控制RS485方向 (sysfs)
class RS485Dir {
    int gpio_;
public:
    RS485Dir(int gpio) : gpio_(gpio) {
        std::ofstream e("/sys/class/gpio/export");
        e << gpio_; e.close();
        usleep(100000);
        std::ofstream d("/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction");
        d << "out"; d.close();
        rx();
    }
    void tx() { std::ofstream v("/sys/class/gpio/gpio" + std::to_string(gpio_) + "/value"); v << "1"; v.close(); }
    void rx() { std::ofstream v("/sys/class/gpio/gpio" + std::to_string(gpio_) + "/value"); v << "0"; v.close(); }
    ~RS485Dir() { std::ofstream e("/sys/class/gpio/unexport"); e << gpio_; e.close(); }
};

int main() {
    // GPIO1_D7 = bank1*32 + 3*8+7 = 32 + 31 = 63
    RS485Dir rs485(63);

    SerialPort serial("/dev/ttyS4");
    MotorCmd cmd;
    MotorData data;

    while(true) {
        cmd.motorType = MotorType::GO_M8010_6;
        data.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        cmd.id   = 0;
        cmd.kp   = 0.0;
        cmd.kd   = 0.01;
        cmd.q    = 0.0;
        cmd.dq   = -6.28 * queryGearRatio(MotorType::GO_M8010_6);
        cmd.tau  = 0.0;

        rs485.tx();
        serial.sendRecv(&cmd, &data);
        rs485.rx();

        std::cout << "q:" << data.q
                  << " dq:" << data.dq
                  << " temp:" << data.temp
                  << " err:" << data.merror
                  << " correct:" << data.correct << std::endl;

        usleep(200);
    }
}
