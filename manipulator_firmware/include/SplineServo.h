#pragma once
#include <Arduino.h>
#include "Config.h"

class SplineServo {
private:
    int pin;
    int channel;
    JointCalibration calib;
    float us_per_rad;

    // Текущее состояние сустава (в радианах и радианах/сек)
    float current_rad;
    float current_vel;
    
    // Коэффициенты кубического полинома (q(t) = a + b*t + c*t^2 + d*t^3)
    float a, b, c, d;
    
    unsigned long motion_start_time;
    unsigned long motion_duration_ms;

    void writeUs(int us);

public:
    SplineServo();
    
    // Новая функция для калибровки
    void setRawUs(int us);

    // Настройка пинов и таймеров (без подачи ШИМ-сигнала!)
    void init(int servo_pin, int pwm_channel, JointCalibration calibration);
    
    // Пробуждение и фиксация в стартовой позиции (Home)
    void wakeUp();

    // Добавление новой точки траектории от ROS 2
    void addWaypoint(float target_rad, float target_vel_rad_s, float duration_sec);
    
    // Шаг математического движка (вызывать в loop на частоте ~200 Гц)
    void update();
    
    float getCurrentRad();
};