import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import Empty
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros
import time

class TeleopManager(Node):
    def __init__(self):
        super().__init__('teleop_manager')
        
        # Publishers to MoveIt Servo
        self.servo_twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.servo_jog_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        # Publisher to Controller for unfolding
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # Publisher for Tool Pose telemetry
        self.pose_pub = self.create_publisher(PoseStamped, '/web_teleop/tool_pose', 10)
        
        # Subscribers from Web UI
        self.web_twist_sub = self.create_subscription(TwistStamped, '/web_teleop/cmd_vel', self.cmd_vel_callback, 10)
        self.web_jog_sub = self.create_subscription(JointJog, '/web_teleop/cmd_jog', self.cmd_jog_callback, 10)
        self.unfold_sub = self.create_subscription(Empty, '/web_teleop/unfold', self.unfold_callback, 10)
        
        from std_msgs.msg import String
        self.mode_sub = self.create_subscription(String, '/web_teleop/mode', self.mode_callback, 10)
        
        # State for continuous publishing
        self.current_twist = Twist()
        self.current_frame_id = 'arm_base_link'
        self.current_jog = JointJog()
        self.active_mode = None  # None, 'TWIST', or 'JOINT_JOG'
        self.target_mode = None
        self.system_mode = 'MANUAL'  # 'MANUAL' or 'AUTO'
        
        # TF2 for telemetry
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timers
        self.servo_timer = self.create_timer(0.02, self.publish_current_command)
        self.telemetry_timer = self.create_timer(0.1, self.publish_telemetry) # 10Hz
        
        # Service client to switch Servo command type
        self.switch_srv = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.switch_in_progress = False
        
        self.get_logger().info("Teleop Manager started.")
        self.zero_msgs_sent = 0

    def publish_telemetry(self):
        try:
            # Look up transform from base to tool0
            t = self.tf_buffer.lookup_transform(
                'arm_base_link',
                'arm_tool0',
                rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'arm_base_link'
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            
            self.pose_pub.publish(pose)
        except tf2_ros.TransformException as ex:
            pass # Suppress warning, tf might not be ready yet

    def request_mode_switch(self, mode):
        if self.switch_in_progress or self.active_mode == mode:
            return
            
        if not self.switch_srv.wait_for_service(timeout_sec=0.1):
            return
            
        self.target_mode = mode
        self.switch_in_progress = True
        
        req = ServoCommandType.Request()
        if mode == 'TWIST':
            req.command_type = ServoCommandType.Request.TWIST
        elif mode == 'JOINT_JOG':
            req.command_type = ServoCommandType.Request.JOINT_JOG
            
        future = self.switch_srv.call_async(req)
        future.add_done_callback(self.switch_callback)

    def switch_callback(self, future):
        self.switch_in_progress = False
        try:
            response = future.result()
            if response.success:
                self.active_mode = self.target_mode
                self.get_logger().info(f"Switched Servo to {self.active_mode} mode.")
            else:
                self.get_logger().warn(f"Failed to switch Servo to {self.target_mode} mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def is_twist_zero(self):
        return (self.current_twist.linear.x == 0.0 and 
                self.current_twist.linear.y == 0.0 and 
                self.current_twist.linear.z == 0.0 and
                self.current_twist.angular.x == 0.0 and
                self.current_twist.angular.y == 0.0 and
                self.current_twist.angular.z == 0.0)
                
    def is_jog_zero(self):
        return len(self.current_jog.velocities) == 0 or all(v == 0.0 for v in self.current_jog.velocities)

    def publish_current_command(self):
        if self.system_mode == 'AUTO':
            # In AUTO mode, ensure Servo is stopped by sending zero commands
            if self.zero_msgs_sent <= 5:
                self.zero_msgs_sent += 1
                if self.active_mode == 'TWIST':
                    stamped_msg = TwistStamped()
                    stamped_msg.header.stamp = self.get_clock().now().to_msg()
                    stamped_msg.header.frame_id = self.current_frame_id
                    self.servo_twist_pub.publish(stamped_msg)
                elif self.active_mode == 'JOINT_JOG':
                    msg = JointJog()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.servo_jog_pub.publish(msg)
            return

        # Determine if we should send a command in MANUAL mode
        if self.active_mode == 'TWIST':
            if self.is_twist_zero():
                self.zero_msgs_sent += 1
                if self.zero_msgs_sent <= 5:
                    stamped_msg = TwistStamped()
                    stamped_msg.header.stamp = self.get_clock().now().to_msg()
                    stamped_msg.header.frame_id = self.current_frame_id
                    stamped_msg.twist = self.current_twist
                    self.servo_twist_pub.publish(stamped_msg)
            else:
                self.zero_msgs_sent = 0
                stamped_msg = TwistStamped()
                stamped_msg.header.stamp = self.get_clock().now().to_msg()
                stamped_msg.header.frame_id = self.current_frame_id
                stamped_msg.twist = self.current_twist
                self.servo_twist_pub.publish(stamped_msg)
                
        elif self.active_mode == 'JOINT_JOG':
            if self.is_jog_zero():
                self.zero_msgs_sent += 1
                if self.zero_msgs_sent <= 5:
                    self.current_jog.header.stamp = self.get_clock().now().to_msg()
                    self.servo_jog_pub.publish(self.current_jog)
            else:
                self.zero_msgs_sent = 0
                self.current_jog.header.stamp = self.get_clock().now().to_msg()
                self.servo_jog_pub.publish(self.current_jog)

    def mode_callback(self, msg):
        from std_msgs.msg import String
        new_mode = msg.data.upper()
        if new_mode in ['MANUAL', 'AUTO'] and self.system_mode != new_mode:
            self.system_mode = new_mode
            self.get_logger().info(f"System mode switched to: {self.system_mode}")
            if self.system_mode == 'AUTO':
                # Force zero messages counter to 0 so we publish 5 zeroes to stop Servo
                self.zero_msgs_sent = 0
                # Clear current commands
                self.current_twist = Twist()
                self.current_jog = JointJog()

    def cmd_vel_callback(self, msg: TwistStamped):
        if self.system_mode == 'AUTO':
            return
        self.current_twist = msg.twist
        self.current_frame_id = msg.header.frame_id
        if not self.is_twist_zero():
            self.request_mode_switch('TWIST')

    def cmd_jog_callback(self, msg: JointJog):
        if self.system_mode == 'AUTO':
            return
        self.current_jog = msg
        if not self.is_jog_zero():
            self.request_mode_switch('JOINT_JOG')

    def unfold_callback(self, msg: Empty):
        self.get_logger().info("Unfolding robot to 'stand' pose...")
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['arm_joint_0', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 1.0, -1.0, 0.0, 0.785, 0.0]
        point.time_from_start = Duration(sec=3, nanosec=0)
        traj.points.append(point)
        self.traj_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
