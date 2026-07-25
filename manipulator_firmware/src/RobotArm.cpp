#include "RobotArm.h"

RobotArm::RobotArm() {}

void RobotArm::init() {
    // Инициализируем каждый сустав из конфигурации
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].init(JOINT_PINS[i], JOINT_CHANNELS[i], ROBOT_CONFIG[i]);
    }
}

void RobotArm::runHomingSequence() {
    Serial.println("\n[СТАРТ] Выполнение последовательной парковки (Homing)...");

    // Этап 1: База (0) и Захват (6)
    Serial.println("-> Этап 1: Активация Базы и Захвата");
    joints[0].wakeUp();
    joints[6].wakeUp();
    delay(1000); // Задержка не блокирует остальной код, так как мы еще в фазе setup

    // Этап 2: Запястье (3, 4, 5)
    Serial.println("-> Этап 2: Выравнивание запястья");
    joints[3].wakeUp();
    joints[4].wakeUp();
    joints[5].wakeUp();
    delay(1000);

    // Этап 3: Плечо (1)
    Serial.println("-> Этап 3: Подъем плеча");
    joints[1].wakeUp();
    delay(1000);

    // Этап 4: Локоть (2)
    Serial.println("-> Этап 4: Фиксация локтя");
    joints[2].wakeUp();
    delay(1000);

    Serial.println("[ГОТОВО] Манипулятор зафиксирован в Home-позиции.");
}

void RobotArm::setTarget(int id, float target_rad, float target_vel, float duration_sec) {
    // Проверка "от дурака", чтобы не обратиться к несуществующему мотору
    if (id >= 0 && id < NUM_JOINTS) {
        joints[id].addWaypoint(target_rad, target_vel, duration_sec);
    }
}

void RobotArm::update() {
    // Централизованное обновление физики всех моторов
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].update();
    }
} 

void RobotArm::setRawUs(int id, int us) {
    if (id >= 0 && id < NUM_JOINTS) {
        joints[id].setRawUs(us);
    }
}

float RobotArm::getCurrentRad(int id) {
    if (id >= 0 && id < NUM_JOINTS) {
        return joints[id].getCurrentRad();
    }
    return 0.0;
}