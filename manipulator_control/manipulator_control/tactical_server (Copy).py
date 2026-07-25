import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
import csv
import os

from manipulator_interfaces.action import ManipulatorGoal
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Quaternion, Pose

def euler_to_quaternion(roll, pitch, yaw):
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)
    qx = math.sin(r/2) * math.cos(p/2) * math.cos(y/2) - math.cos(r/2) * math.sin(p/2) * math.sin(y/2)
    qy = math.cos(r/2) * math.sin(p/2) * math.cos(y/2) + math.sin(r/2) * math.cos(p/2) * math.sin(y/2)
    qz = math.cos(r/2) * math.cos(p/2) * math.sin(y/2) - math.sin(r/2) * math.sin(p/2) * math.cos(y/2)
    qw = math.cos(r/2) * math.cos(p/2) * math.cos(y/2) + math.sin(r/2) * math.sin(p/2) * math.sin(y/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class TacticalServer(Node):
    def __init__(self):
        super().__init__('tactical_server')
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, ManipulatorGoal, 'manipulator_action',
            self.execute_callback, callback_group=self.cb_group)

        self.move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)

        self.get_logger().info('🔥 Тактический сервер запущен! Ожидание команд (включая CSV)...')

    def _send_pose_to_moveit(self, target_pose, speed_scale, ignore_orientation=False):
        """Внутренняя функция отправки координат в MoveIt"""
        while not self.move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Ждем /move_action...')

        goal_msg = MoveGroup.Goal()
        mpr = MotionPlanRequest()
        mpr.group_name = "arm"
        mpr.num_planning_attempts = 10               
        mpr.allowed_planning_time = 5.0              
        
        mpr.max_velocity_scaling_factor = speed_scale
        mpr.max_acceleration_scaling_factor = speed_scale

        constraints = Constraints()
        
        # Позиция
        pc = PositionConstraint()
        pc.header.frame_id = "arm_base_link"
        pc.link_name = "arm_tool0"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.03] 
        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # Ориентация
        if not ignore_orientation:
            oc = OrientationConstraint()
            oc.header.frame_id = "arm_base_link"
            oc.link_name = "arm_tool0"
            oc.orientation = target_pose.orientation
            oc.absolute_x_axis_tolerance = 0.15 
            oc.absolute_y_axis_tolerance = 0.15
            oc.absolute_z_axis_tolerance = 0.15
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        mpr.goal_constraints.append(constraints)
        goal_msg.request = mpr
        
        goal_msg.planning_options.plan_only = False 
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        send_goal_future = self.move_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.05)
            
        action_handle = send_goal_future.result()
        if not action_handle.accepted:
            return False, "Отклонено MoveIt"

        result_future = action_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)
            
        result_obj = result_future.result().result
        
        if result_obj.error_code.val == 1:
            return True, "Успех"
        else:
            return False, f"Ошибка кода: {result_obj.error_code.val}"

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f'Получена команда! Режим: {req.mode}')

        # Единый расчет скорости
        speed = req.speed_scaling if req.speed_scaling > 0.0 else 0.1
        speed = max(0.01, min(1.0, speed))

        feedback = ManipulatorGoal.Feedback()

        # ==========================================
        # РЕЖИМ 1: ЕДИНИЧНАЯ ТОЧКА
        # ==========================================
        if req.mode in ["POSE", "POSITION_ONLY"]:
            if req.target_pose.orientation.w == 1.0 and req.target_pose.orientation.x == 0.0 and req.target_pose.orientation.y == 0.0 and req.target_pose.orientation.z == 0.0:
                final_pose = req.target_pose
            else:
                final_pose = req.target_pose
                final_pose.orientation = euler_to_quaternion(
                    req.target_pose.orientation.x, 
                    req.target_pose.orientation.y, 
                    req.target_pose.orientation.z
                )

            ignore_ori = (req.mode == "POSITION_ONLY")
            success, msg = self._send_pose_to_moveit(final_pose, speed, ignore_ori)
            
            res = ManipulatorGoal.Result()
            res.success = success
            res.message = msg
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return res

        # ==========================================
        # РЕЖИМ 2: ШАБЛОН ИЗ CSV
        # ==========================================
        elif req.mode == "CSV":
            self.get_logger().info(f'Открываем шаблон: {req.csv_path}')
            
            if not os.path.exists(req.csv_path):
                self.get_logger().error('Файл не найден!')
                goal_handle.abort()
                return ManipulatorGoal.Result(success=False, message="Файл не найден")

            waypoints = []
            with open(req.csv_path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None) # Пропускаем заголовки X, Y, Z...
                for row in reader:
                    if len(row) >= 6:
                        waypoints.append([float(x) for x in row[:6]])
                    elif len(row) >= 3: # Если в файле только X, Y, Z
                        waypoints.append([float(row[0]), float(row[1]), float(row[2]), 0.0, 0.0, 0.0])

            total_points = len(waypoints)
            self.get_logger().info(f'Загружено {total_points} точек. Начинаем проход...')
            
            for i, wp in enumerate(waypoints):
                feedback.current_state = f"Выполнение точки {i+1}/{total_points}"
                feedback.progress = float(i) / total_points
                goal_handle.publish_feedback(feedback)
                
                self.get_logger().info(f'-> Точка {i+1}: X:{wp[0]:.3f}, Y:{wp[1]:.3f}, Z:{wp[2]:.3f}')
                
                pose = Pose()
                pose.position.x = wp[0]
                pose.position.y = wp[1]
                pose.position.z = wp[2]
                pose.orientation = euler_to_quaternion(wp[3], wp[4], wp[5])
                
                # Для CSV пока жестко выдерживаем ориентацию. Если хочешь - можем вынести в параметры
                success, msg = self._send_pose_to_moveit(pose, speed, ignore_orientation=False)
                
                if not success:
                    self.get_logger().error(f'Застряли на точке {i+1}! Отмена шаблона. Причина: {msg}')
                    goal_handle.abort()
                    return ManipulatorGoal.Result(success=False, message=f"Сбой на точке {i+1}")

            self.get_logger().info('✅ Шаблон CSV успешно выполнен!')
            goal_handle.succeed()
            return ManipulatorGoal.Result(success=True, message="CSV завершен")

        # ==========================================
        else:
            self.get_logger().warning(f'Режим {req.mode} не распознан.')
            goal_handle.abort()
            return ManipulatorGoal.Result(success=False, message="Неизвестный режим")

def main(args=None):
    rclpy.init(args=args)
    server = TacticalServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()