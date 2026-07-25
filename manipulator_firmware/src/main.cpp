#include <Arduino.h>
#include "RobotArm.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>

RobotArm arm;

// Подписчик (Слушает команды от ROS)
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg_cmd;
float msg_buffer[4]; 

// Издатель (Отправляет координаты в ROS)
rcl_publisher_t publisher;
sensor_msgs__msg__JointState joint_state_msg;

// Статическая память для массива JointState (чтобы не использовать malloc)
rosidl_runtime_c__String joint_names[NUM_JOINTS];
double joint_positions[NUM_JOINTS];
const char* joint_name_strings[] = {"arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_clamp"};

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg_in = (const std_msgs__msg__Float32MultiArray *)msgin;
    
    if (msg_in->data.size == 4) {
        int id = (int)msg_in->data.data[0];
        float target_rad = msg_in->data.data[1];
        float target_vel = msg_in->data.data[2];
        float duration = msg_in->data.data[3];
        arm.setTarget(id, target_rad, target_vel, duration);
    } 
    else if (msg_in->data.size == 2) {
        int id = (int)msg_in->data.data[0];
        int raw_us = (int)msg_in->data.data[1];
        arm.setRawUs(id, raw_us);
    }
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    arm.init();
    arm.runHomingSequence();

    allocator = rcl_get_default_allocator();
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { delay(100); }
    rmw_uros_sync_session(1000);
    
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "robot_arm_node", "", &support);

    // --- НАСТРОЙКА ПОДПИСЧИКА КОМАНД (Добавлен префикс arm/) ---
    msg_cmd.data.capacity = 4;
    msg_cmd.data.data = msg_buffer;
    msg_cmd.data.size = 0;
    rclc_subscription_init_best_effort(
        &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm/servo_cmd"
    );

    // --- НАСТРОЙКА ИЗДАТЕЛЯ СОСТОЯНИЙ (Возвращаем best_effort для скорости и префикс arm/) ---
    rclc_publisher_init_best_effort(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "arm/joint_states_raw"
    );
    

    //   rclc_publisher_init_best_effort(
 //       &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"
 //   );

    // Привязываем статическую память к сообщению JointState
    joint_state_msg.name.capacity = NUM_JOINTS;
    joint_state_msg.name.size = NUM_JOINTS;
    joint_state_msg.name.data = joint_names;

    joint_state_msg.position.capacity = NUM_JOINTS;
    joint_state_msg.position.size = NUM_JOINTS;
    joint_state_msg.position.data = joint_positions;

    // Жестко прописываем имена суставов (должны совпадать с URDF)
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_state_msg.name.data[i].data = (char*)joint_name_strings[i];
        joint_state_msg.name.data[i].size = strlen(joint_name_strings[i]);
        joint_state_msg.name.data[i].capacity = joint_state_msg.name.data[i].size + 1;
    }

    // Экзекутор только для подписчика (издателю он не нужен)
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_cmd, &subscription_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); 
    arm.update();
    
    // Публикация координат с частотой 20 Гц (каждые 50 мс)
    static unsigned long last_pub = 0;
    if (millis() - last_pub > 50) {
        
        // Считываем текущие радианы из сплайн-генератора
        for (int i = 0; i < NUM_JOINTS; i++) {
            joint_state_msg.position.data[i] = (double)arm.getCurrentRad(i);
        }
        
        // Добавляем timestamp для RViz (очень важно для TF)
        int64_t time_ns = rmw_uros_epoch_nanos();
        joint_state_msg.header.stamp.sec = time_ns / 1000000000;
        joint_state_msg.header.stamp.nanosec = time_ns % 1000000000;

        rcl_publish(&publisher, &joint_state_msg, NULL);
        last_pub = millis();
    }

    delay(4); 
}