import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import csv
import os
import time
import math
import threading
import json

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from manipulator_interfaces.action import ManipulatorGoal, HubCommand

from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, OrientationConstraint, RobotState
from sensor_msgs.msg import JointState
import tf2_ros

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
    q = Pose().orientation
    q.x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    q.y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    q.z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    q.w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return q

def rotate_q_z(q_base, angle):
    q_rot = Pose().orientation
    q_rot.x = 0.0
    q_rot.y = 0.0
    q_rot.z = math.sin(angle/2.0)
    q_rot.w = math.cos(angle/2.0)
    return quaternion_multiply(q_base, q_rot)

class ControlHub(Node):
    def __init__(self):
        super().__init__('control_hub')
        self.cb_group = ReentrantCallbackGroup()
        
        # Action Server to receive commands from terminal/ROS nodes
        self._action_server = ActionServer(
            self, HubCommand, 'hub_command',
            self.execute_callback, callback_group=self.cb_group)
            
        # Web UI Topic Bridge
        self.web_cmd_sub = self.create_subscription(
            String, '/web_teleop/hub_request', self.web_cmd_callback, 10, callback_group=self.cb_group)
        self.web_status_pub = self.create_publisher(
            String, '/web_teleop/hub_status', 10)
        self.is_executing = False
        
        # Action Client to send goals to Tactical Server
        self.action_client = ActionClient(self, ManipulatorGoal, 'manipulator_action', callback_group=self.cb_group)
        
        # IK/FK Clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk', callback_group=self.cb_group)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10, callback_group=self.cb_group)
        
        self.get_logger().info('🧠 CONTROL HUB (МОЗГ) ЗАПУЩЕН И ЖДЕТ КОМАНД.')

    def joint_states_callback(self, msg):
        self.current_joint_state = msg

    def get_current_tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('arm_base_link', 'arm_tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception as e:
            return None

    def send_goal_and_wait(self, mode, poses, ignore_ori, speed, pos_tol, ori_tol):
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Ждем Тактический Сервер...')
            
        goal_msg = ManipulatorGoal.Goal()
        goal_msg.mode = mode
        goal_msg.poses = poses
        goal_msg.ignore_orientation = ignore_ori
        goal_msg.speed = speed
        goal_msg.pos_tolerance = pos_tol
        goal_msg.ori_tolerance = ori_tol

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_goal_future.done(): time.sleep(0.05)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done(): time.sleep(0.05)
        return result_future.result().result.success

    def get_ik_for_point(self, x, y, z, target_q, seed_joints):
        req_ik = GetPositionIK.Request()
        req_ik.ik_request.group_name = "arm"
        req_ik.ik_request.avoid_collisions = True
        req_ik.ik_request.robot_state = seed_joints
            
        req_ik.ik_request.pose_stamped.header.frame_id = "arm_base_link"
        req_ik.ik_request.pose_stamped.pose.position.x = float(x)
        req_ik.ik_request.pose_stamped.pose.position.y = float(y)
        req_ik.ik_request.pose_stamped.pose.position.z = float(z)
        req_ik.ik_request.pose_stamped.pose.orientation = target_q
        
        oc = OrientationConstraint()
        oc.header.frame_id = "arm_base_link"
        oc.link_name = "arm_tool0"
        oc.orientation = target_q
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 0.2 
        oc.weight = 1.0
        req_ik.ik_request.constraints.orientation_constraints.append(oc)

        future_ik = self.ik_client.call_async(req_ik)
        while rclpy.ok() and not future_ik.done(): time.sleep(0.05)
        res_ik = future_ik.result()
        
        if res_ik.error_code.val != 1:
            return None, None

        req_fk = GetPositionFK.Request()
        req_fk.header.frame_id = "arm_base_link"
        req_fk.fk_link_names = ["arm_tool0"]
        req_fk.robot_state = res_ik.solution

        future_fk = self.fk_client.call_async(req_fk)
        while rclpy.ok() and not future_fk.done(): time.sleep(0.05)
        res_fk = future_fk.result()

        if res_fk.error_code.val != 1:
            return None, None

        return res_fk.pose_stamped[0].pose, res_ik.solution

    # ==========================================
    # WEB UI BRIDGE
    # ==========================================
    def publish_web_status(self, state, progress, success=None, message=""):
        msg = String()
        data = {
            "state": state,
            "progress": progress
        }
        if success is not None:
            data["done"] = True
            data["success"] = success
            data["message"] = message
        else:
            data["done"] = False
            
        msg.data = json.dumps(data)
        self.web_status_pub.publish(msg)

    def web_cmd_callback(self, msg):
        if self.is_executing:
            self.publish_web_status("Ошибка: Хаб занят", 0, False, "Другая задача уже выполняется")
            return
            
        try:
            req = json.loads(msg.data)
            cmd = req.get("command")
            file_path = req.get("file_path")
            
            if cmd == "EXECUTE_CSV":
                threading.Thread(target=self.execute_csv_task_web, args=(file_path,)).start()
            else:
                self.publish_web_status("Ошибка", 0, False, "Неизвестная команда")
        except Exception as e:
            self.publish_web_status("Ошибка парсинга", 0, False, str(e))

    def execute_csv_task_web(self, csv_path):
        self.is_executing = True
        
        def update_status(state, prog):
            self.publish_web_status(state, prog)
            
        success, msg = self._run_csv_logic(csv_path, update_status)
        
        if success:
            self.publish_web_status("Успех", 100, True, msg)
        else:
            self.publish_web_status("Ошибка", 100, False, msg)
            
        self.is_executing = False

    # ==========================================
    # ACTION SERVER (Terminal / ROS)
    # ==========================================
    def execute_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f'Hub Action received command: {req.command}')
        
        if self.is_executing:
            goal_handle.abort()
            res = HubCommand.Result()
            res.success = False
            res.message = "Хаб занят"
            return res
            
        self.is_executing = True
        
        feedback = HubCommand.Feedback()
        result = HubCommand.Result()
        
        if req.command == "EXECUTE_CSV":
            def update_status(state, prog):
                feedback.current_state = state
                feedback.progress = prog
                goal_handle.publish_feedback(feedback)
                
            success, msg = self._run_csv_logic(req.file_path, update_status)
            
            if success:
                goal_handle.succeed()
                result.success = True
                result.message = msg
            else:
                goal_handle.abort()
                result.success = False
                result.message = msg
        else:
            goal_handle.abort()
            result.success = False
            result.message = "Неизвестная команда"
            
        self.is_executing = False
        return result

    # ==========================================
    # CORE LOGIC
    # ==========================================
    def _run_csv_logic(self, csv_path, status_cb):
        status_cb("Чтение файла...", 10)
        
        if not os.path.exists(csv_path):
            self.get_logger().error(f"Файл не найден: {csv_path}")
            return False, "Файл не найден"

        raw_points = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)
            for row in reader:
                if len(row) >= 3:
                    raw_points.append([float(row[0]), float(row[1]), float(row[2])])

        if not raw_points:
            return False, "Файл пуст"

        status_cb("Интеллектуальный поиск ИК цепи...", 20)

        # Wait for TF and joint states
        current_pose = self.get_current_tf_pose()
        while current_pose is None and rclpy.ok():
            time.sleep(0.1)
            current_pose = self.get_current_tf_pose()

        while self.current_joint_state is None and rclpy.ok():
            time.sleep(0.1)

        base_q = current_pose.orientation
        start_robot_state = RobotState()
        start_robot_state.joint_state = self.current_joint_state

        variations = [0.0, 0.3, -0.3, 0.7, -0.7, 1.57, -1.57, 2.3, -2.3, 3.14]
        valid_poses = None
        best_angle = 0.0

        for angle in variations:
            self.get_logger().info(f'🔎 Пробуем построить цепь (доворот кисти {angle:.2f} рад)...')
            test_q = rotate_q_z(base_q, angle)
            
            temp_poses = []
            chain_q = test_q
            chain_joints = start_robot_state
            
            success_ik = True
            for i, pt in enumerate(raw_points):
                pose, joints = self.get_ik_for_point(pt[0], pt[1], pt[2], target_q=chain_q, seed_joints=chain_joints)
                
                if not pose:
                    success_ik = False
                    break
                    
                temp_poses.append(pose)
                chain_q = pose.orientation 
                chain_joints = joints
                
            if success_ik:
                valid_poses = temp_poses
                best_angle = angle
                break

        if not valid_poses:
            self.get_logger().error('❌ Фигура физически недостижима.')
            return False, "Фигура физически недостижима из текущей позы"

        status_cb("Выполнение: Подлет к старту...", 40)

        success_move = self.send_goal_and_wait(mode="POSE", poses=[valid_poses[0]], ignore_ori=False, speed=0.8, pos_tol=0.05, ori_tol=0.2)
        if not success_move:
            self.get_logger().error('Не смогли доехать до первой точки!')
            return False, "Сбой подлета"

        time.sleep(1.0)
        
        status_cb("Выполнение: Отрисовка пути...", 60)

        success_draw = self.send_goal_and_wait(mode="CARTESIAN", poses=valid_poses, ignore_ori=False, speed=0.3, pos_tol=0.05, ori_tol=0.2)
        
        if success_draw:
            self.get_logger().info('🎯 ФИГУРА ИДЕАЛЬНО ОТРИСОВАНА!')
            return True, "Успешно отрисовано"
        else:
            self.get_logger().error('⚠️ ОШИБКА ДЕКАРТОВА ПУТИ.')
            return False, "Сбой декартова пути"

def main(args=None):
    rclpy.init(args=args)
    hub = ControlHub()
    try:
        rclpy.spin(hub, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        hub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()