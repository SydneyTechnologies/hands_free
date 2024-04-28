import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from .ArduinoController import ArduinoController
from .Odometry import Odometry as Odom
from std_msgs.msg import String

class Wheel:
    ticksPerRotation = 90
    wheelRadius = 0.0475
    wheelSeperation = 0.36
    ticksPerRadian = 14.3

class Quaternion:
    @staticmethod
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q



class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.arduino = ArduinoController()

        # Initialize class variables
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_state_publisher = self.create_publisher(Odometry, "odom", 1)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.wheel_state_publisher = self.create_publisher(String, 'wheel_states', 1)
        self.prev_l_ticks = 0
        self.prev_r_ticks = 0

        self.timer_period = 0.05  # seconds
        self.odom = Odom(Wheel.wheelRadius, Wheel.wheelSeperation, Wheel.ticksPerRotation)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.publish_robot_state)
        self.odomData = self.odom.updateOdometry(0, 0, 1)


    def publish_robot_state(self):
        tf_msg = TransformStamped()
        odom_msg = Odometry()
        joint_state_msg = JointState()
        wheel_state_msg = String()
        data = self.arduino.read()


        if data.startswith("@"):
            # Extract Data
            rawData = data.split("/")
            l_ticks = int(rawData[0][1:])  
            r_ticks = int(rawData[1])
            # theta = float(rawData[2])
            wheel_state_msg.data = data
            self.wheel_state_publisher.publish(wheel_state_msg)

            # print(theta)

            dt = (self.get_clock().now().nanoseconds - self.start_time.nanoseconds)/math.pow(10, 9)
            self.start_time = self.get_clock().now()
            # try:
            #     self.odomData = self.odom.updateOdometry(l_ticks, r_ticks, dt, theta)
            # except Exception as err:
            self.odomData = self.odom.updateOdometry(r_ticks, l_ticks, dt)


        # Publish to JOINT_STATES
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_link' 
        joint_state_msg.name = ["back_left_wheel_joint", "back_right_wheel_joint"]
        joint_state_msg.position = self.odomData.angular_positions
        joint_state_msg.velocity = self.odomData.angular_velocities
        self.joint_state_publisher.publish(joint_state_msg)


        # Publish to TF 
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.odomData.robot_x
        tf_msg.transform.translation.y = self.odomData.robot_y
        tf_msg.transform.translation.z = 0.0
        q = Quaternion.quaternion_from_euler(0, 0, self.odomData.robot_theta)
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)


        # Publish to Odom Topic 
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = self.odomData.robot_x
        odom_msg.pose.pose.position.y = self.odomData.robot_y
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y =  q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = self.odomData.robot_linear
        odom_msg.twist.twist.angular.z = self.odomData.robot_angular
        self.odom_state_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    state_publisher = StatePublisher()

    rclpy.spin(state_publisher)

    state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
