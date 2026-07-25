import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math

from manipulator_interfaces.action import ManipulatorGoal
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

class TacticalServer(Node):
    def __init__(self):
        super().__init__('tactical_server')
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, ManipulatorGoal, 'manipulator_action',
            self.execute_callback, callback_group=self.cb_group)

        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb_group)
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.cb_group)

        self.get_logger().info('🔥 ТАКТИЧЕСКИЙ СЕРВЕР (МЫШЦЫ) ЗАПУЩЕН.')

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        speed = max(0.01, min(1.0, req.speed if req.speed > 0.0 else 0.1))
        
        # Читаем допуски из запроса (если 0, ставим дефолт)
        pos_tol = req.pos_tolerance if req.pos_tolerance > 0.0 else 0.03
        ori_tol = req.ori_tolerance if req.ori_tolerance > 0.0 else 0.15
        
        self.get_logger().info(f'Команда: Режим={req.mode}, Точек={len(req.poses)}, Игнор.углов={req.ignore_orientation}')
        self.get_logger().info(f'Параметры: Скорость={speed:.2f}, Допуск поз.={pos_tol}м, Допуск ориен.={ori_tol}рад')

        if not req.poses:
            self.get_logger().error("Массив координат пуст!")
            goal_handle.abort()
            return ManipulatorGoal.Result(success=False, message="Пустой массив")

        # ==========================================
        # РЕЖИМ 1: PTP (ПОЛЕТ В ОДНУ ТОЧКУ)
        # ==========================================
        if req.mode == "POSE":
            while not self.move_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Ждем /move_action...')

            goal_msg = MoveGroup.Goal()
            mpr = MotionPlanRequest()
            mpr.group_name = "arm"
            mpr.num_planning_attempts = 10               
            mpr.allowed_planning_time = 5.0              
            mpr.max_velocity_scaling_factor = speed
            mpr.max_acceleration_scaling_factor = speed

            constraints = Constraints()
            
            # Позиция (применяем динамический допуск)
            pc = PositionConstraint()
            pc.header.frame_id = "arm_base_link"
            pc.link_name = "arm_tool0"
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [pos_tol] 
            pc.constraint_region.primitives.append(primitive)
            pc.constraint_region.primitive_poses.append(req.poses[0])
            pc.weight = 1.0
            constraints.position_constraints.append(pc)
            
            # Ориентация (применяем динамический допуск)
            if not req.ignore_orientation:
                oc = OrientationConstraint()
                oc.header.frame_id = "arm_base_link"
                oc.link_name = "arm_tool0"
                oc.orientation = req.poses[0].orientation
                oc.absolute_x_axis_tolerance = ori_tol 
                oc.absolute_y_axis_tolerance = ori_tol
                oc.absolute_z_axis_tolerance = ori_tol
                oc.weight = 1.0
                constraints.orientation_constraints.append(oc)

            mpr.goal_constraints.append(constraints)
            goal_msg.request = mpr
            goal_msg.planning_options.plan_only = False 
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
            
            send_goal_future = self.move_client.send_goal_async(goal_msg)
            while rclpy.ok() and not send_goal_future.done(): time.sleep(0.05)
                
            action_handle = send_goal_future.result()
            if not action_handle.accepted:
                goal_handle.abort()
                return ManipulatorGoal.Result(success=False, message="Отклонено MoveIt")

            res_future = action_handle.get_result_async()
            while rclpy.ok() and not res_future.done(): time.sleep(0.05)
                
            if res_future.result().result.error_code.val == 1:
                goal_handle.succeed()
                return ManipulatorGoal.Result(success=True, message="Достигнуто")
            else:
                goal_handle.abort()
                return ManipulatorGoal.Result(success=False, message="Сбой пути")

        # ==========================================
        # РЕЖИМ 2: CARTESIAN (РОВНАЯ ЛИНИЯ ПО МАССИВУ)
        # ==========================================
        elif req.mode == "CARTESIAN":
            while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Ждем /compute_cartesian_path...')

            req_cart = GetCartesianPath.Request()
            req_cart.header.frame_id = 'arm_base_link'
            req_cart.group_name = 'arm'
            req_cart.link_name = 'arm_tool0'
            req_cart.waypoints = req.poses
            req_cart.max_step = 0.01          
            req_cart.jump_threshold = 0.0     
            req_cart.avoid_collisions = True
            req_cart.max_velocity_scaling_factor = speed
            req_cart.max_acceleration_scaling_factor = speed

            future_cart = self.cartesian_client.call_async(req_cart)
            while rclpy.ok() and not future_cart.done(): time.sleep(0.05)
            
            res_cart = future_cart.result()
            
            if res_cart.fraction < 0.9:
                self.get_logger().error(f'Линия недостижима. Просчитано {res_cart.fraction*100:.1f}%')
                goal_handle.abort()
                return ManipulatorGoal.Result(success=False, message="Линия выходит за лимиты")

                   # Правильное масштабирование времени (без переполнения наносекунд)
            #for point in res_cart.solution.joint_trajectory.points:
            #    total_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            #    scaled_time = total_time / speed
            #    point.time_from_start.sec = int(scaled_time)
            #    point.time_from_start.nanosec = int((scaled_time - int(scaled_time)) * 1e9)

            

            while not self.execute_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Ждем /execute_trajectory...')
                
            goal_exec = ExecuteTrajectory.Goal()
            goal_exec.trajectory = res_cart.solution
            
            future_exec = self.execute_client.send_goal_async(goal_exec)
            while rclpy.ok() and not future_exec.done(): time.sleep(0.05)
            
            action_exec_handle = future_exec.result()
            if not action_exec_handle.accepted:
                goal_handle.abort()
                return ManipulatorGoal.Result(success=False, message="Отклонено Execute")

            res_exec_future = action_exec_handle.get_result_async()
            while rclpy.ok() and not res_exec_future.done(): time.sleep(0.05)
            
            if res_exec_future.result().result.error_code.val == 1:
                goal_handle.succeed()
                return ManipulatorGoal.Result(success=True, message="Линия отрисована")
            else:
                goal_handle.abort()
                return ManipulatorGoal.Result(success=False, message="Ошибка выполнения")

        else:
            goal_handle.abort()
            return ManipulatorGoal.Result(success=False, message="Неизвестный режим")

def main(args=None):
    rclpy.init(args=args)
    server = TacticalServer()
    try: rclpy.spin(server, executor=MultiThreadedExecutor())
    except KeyboardInterrupt: pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()