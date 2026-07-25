import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

class TrajectoryActionServer(Node):
    def __init__(self):
        # Node name. The namespace will be set during launch.
        super().__init__('trajectory_bridge')
        
        self.cb_group = ReentrantCallbackGroup()

        # Joint names now have the prefix 'arm_' (as in our URDF)
        self.joint_map = {
            'arm_joint_0': 0.0, 'arm_joint_1': 1.0, 'arm_joint_2': 2.0,
            'arm_joint_3': 3.0, 'arm_joint_4': 4.0, 'arm_joint_5': 5.0
        }

        # Relative topic. ROS will prepend /arm/ in front. (For ESP32)
        self.publisher_ = self.create_publisher(Float32MultiArray, '/arm/servo_cmd', 10)
        
        # 2. UPDATED: Buffer topic for the mixer! (Was /joint_states)
        self.js_publisher = self.create_publisher(JointState, '/arm_joint_states', 10)
        
        # 3. Relative topic. ROS will prepend /arm/ in front. (From ESP32)
        self.js_subscriber = self.create_subscription(
            JointState,
            '/arm/joint_states_raw', 
            self.joint_states_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group) 

        # 4. Relative topic. ROS will prepend /arm/ in front. (From MoveIt)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            self.execute_callback,
            callback_group=self.cb_group)

        self.get_logger().info("🔥 Изолированный мост манипулятора запущен!")

    def joint_states_callback(self, msg):
        # ESP32 sends raw data. We set the exact computer time and publish to the global bus.
        msg.header.stamp = self.get_clock().now().to_msg()
        self.js_publisher.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('🤖 Траектория принята от MoveIt...')
        trajectory = goal_handle.request.trajectory
        prev_time = 0.0

        for point in trajectory.points:
            current_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            duration = current_time - prev_time
            if duration <= 0.01:
                duration = 0.05

            for i, joint_name in enumerate(trajectory.joint_names):
                if joint_name in self.joint_map:
                    cmd_msg = Float32MultiArray()
                    cmd_msg.data = [self.joint_map[joint_name], float(point.positions[i]), 0.0, float(duration)]
                    self.publisher_.publish(cmd_msg)

            time.sleep(duration)
            prev_time = current_time

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main(args=None):
    rclpy.init(args=args)
    server = TrajectoryActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()