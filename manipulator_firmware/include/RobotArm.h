#pragma once
#include "Config.h"
#include "SplineServo.h"

class RobotArm {
private:
    // Массив всех наших сервоприводов скрыт внутри класса
    SplineServo joints[NUM_JOINTS];

public:
    RobotArm();
    
    // Базовые функции управления манипулятором
    void init();
    void runHomingSequence();
    void setTarget(int id, float target_rad, float target_vel, float duration_sec);
    void update();
    void setRawUs(int id, int us);
    float getCurrentRad(int id);
};