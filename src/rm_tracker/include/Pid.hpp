#ifndef PIDE
#define PIDE
#include <iostream>
// 老登遗留: 未经过使用
class PID {
    friend std::ostream& operator<<(std::ostream& o, PID a);
private:
    double kp;
    double ki;
    double kd;
    double dt;
    double previous_error;
    double integral;
public:
    PID(double kp, double ki, double kd, double dt = 1) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->dt = dt;
        this->previous_error = 0;
        this->integral = 0;
    }
    PID() {
        this->kp = 0.6;
        this->ki = 0.1;
        this->kd = 0.01;
        this->dt = 1;
        this->previous_error = 0;
        this->integral = 0;
    }

    double Update(double error, double dt = 0) {
        if (dt == 0)
            dt = this->dt;
        double proportional = error;
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        double output = kp * proportional + ki * integral + kd * derivative;
        previous_error = error;
        return output;
    }

    void Clear() {
        this->previous_error = 0;
        this->integral = 0;
    }
};
//全局函数
inline std::ostream& operator<<(std::ostream& o, PID a) {
    o << '(' << a.kp << ',' << a.ki << "," << a.kd << "," << a.dt << ')';
    return o;
}
#endif