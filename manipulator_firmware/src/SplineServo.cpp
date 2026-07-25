#include "SplineServo.h"

SplineServo::SplineServo() {} 

void SplineServo::init(int servo_pin, int pwm_channel, JointCalibration calibration) {
    pin = servo_pin;
    channel = pwm_channel;
    calib = calibration;
    
    // Предрасчет коэффициента перевода: микросекунды на 1 радиан
    us_per_rad = (calib.max_us - calib.center_us) / calib.max_rad;

    current_rad = 0.0;
    current_vel = 0.0;
    a = b = c = d = 0.0;
    motion_duration_ms = 0;

    // Настройка 16-битного таймера ESP32 (для Core 2.x)
    ledcSetup(channel, 50, 16); 
    ledcAttachPin(pin, channel);
    
    // ВНИМАНИЕ: Мы не подаем сигнал здесь, чтобы избежать рывка при включении питания.
    // Выход остается в "воздухе" (0 скважность), пока не вызовут wakeUp().
    ledcWrite(channel, 0); 
}

void SplineServo::wakeUp() {
    // Устанавливаем математику в стартовую позицию (задана в Config.h)
    current_rad = calib.home_rad;
    current_vel = 0.0;
    
    a = current_rad; 
    b = 0.0;
    c = 0.0;
    d = 0.0;
    motion_duration_ms = 0;

    // Вычисляем ШИМ для стартовой позы и жестко фиксируем мотор
    int us = calib.center_us + (int)(current_rad * us_per_rad * calib.direction);
    writeUs(us);
}

void SplineServo::writeUs(int us) {
    // Жесткая аппаратная защита от выхода за лимиты (чтобы не сломать механику)
    if (us < calib.limit_min_us) us = calib.limit_min_us;
    if (us > calib.limit_max_us) us = calib.limit_max_us;
    
    // 16 бит = 65535. Период 50Гц = 20000 мкс.
    uint32_t duty = (us * 65535) / 20000;
    ledcWrite(channel, duty);
}

void SplineServo::addWaypoint(float target_rad, float target_vel, float duration_sec) {
    if (duration_sec <= 0.001) { // Защита от деления на ноль (мгновенный прыжок)
        current_rad = target_rad;
        current_vel = target_vel;
        return;
    }

    float q0 = current_rad;
    float v0 = current_vel;
    float q1 = target_rad;
    float v1 = target_vel;
    float T = duration_sec;

    // Вычисление коэффициентов кубического сплайна
    a = q0;
    b = v0;
    c = (3.0 * (q1 - q0) / (T * T)) - ((2.0 * v0 + v1) / T);
    d = (2.0 * (q0 - q1) / (T * T * T)) + ((v0 + v1) / (T * T));

    motion_start_time = millis();
    motion_duration_ms = (unsigned long)(duration_sec * 1000.0);
}

void SplineServo::update() {
    if (motion_duration_ms == 0) return;

    unsigned long elapsed = millis() - motion_start_time;

    if (elapsed < motion_duration_ms) {
        float t = (float)elapsed / 1000.0; // Текущее время интерполяции в секундах
        
        // Расчет текущего угла и скорости по полиному
        current_rad = a + b * t + c * t * t + d * t * t * t;
        current_vel = b + 2.0 * c * t + 3.0 * d * t * t;
    } else {
        // Время вышло - фиксируемся на конечной точке, скорость гасим в ноль
        float T = (float)motion_duration_ms / 1000.0;
        current_rad = a + b * T + c * T * T + d * T * T * T;
        current_vel = 0.0;
        motion_duration_ms = 0; 
    }

    // Переводим радианы обратно в микросекунды с учетом направления
    int us = calib.center_us + (int)(current_rad * us_per_rad * calib.direction);
    writeUs(us);
}

void SplineServo::setRawUs(int us) {
    motion_duration_ms = 0; // Останавливаем расчет сплайна

    // Обрезаем сигнал по лимитам из Config.h для защиты металла
    if (us < calib.limit_min_us) us = calib.limit_min_us;
    if (us > calib.limit_max_us) us = calib.limit_max_us;

    // Мгновенно подаем ШИМ на мотор
    uint32_t duty = (us * 65535) / 20000;
    ledcWrite(channel, duty);

    // ВАЖНО: Синхронизируем математику сплайна. 
    // Вычисляем, какому углу соответствуют эти микросекунды, 
    // чтобы следующая плавная команда от ROS началась ровно с этого места без рывка.
    current_rad = (us - calib.center_us) / (us_per_rad * calib.direction);
    current_vel = 0.0;
    
    a = current_rad;
    b = 0.0;
    c = 0.0;
    d = 0.0;
}

float SplineServo::getCurrentRad() {
    return current_rad;
}