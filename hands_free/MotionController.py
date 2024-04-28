import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from .ArduinoController import ArduinoController
from .StatePublisher import Wheel
from std_msgs.msg import String


class MotionController(Node):

    def __init__(self):
        super().__init__('motion_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.sendCommandVelocities,
            1)
        self.wheel_data_subscriber = self.create_subscription(
            String,
            '/wheel_states',
            self.get_wheel_data,
            1)
        
        # Initialize wheel ticks and Arduino controller
        self.prev_l_ticks = 0
        self.prev_r_ticks = 0
        self.arduino = ArduinoController()

        # Keep track of time for delta time calculation
        self.start_time = self.get_clock().now()
        
        # Track stop state
        self.stopState = False

    def get_wheel_data(self, msg):
        data = msg.data
        rawData = data.split("/")
        
        # Parse wheel tick counts
        self.prev_l_ticks = int(rawData[0][1:])  # left wheel
        self.prev_r_ticks = int(rawData[1])  # right wheel

    def sendCommandVelocities(self, msg):
        dt = 0.05

        # Update the start time for the next iteration
        # self.start_time = self.get_clock().now()

        # Get linear and angular velocities
        linear_vel = msg.linear.x 
        angular_vel = msg.angular.z

        # Convert velocities to ticks
        left_velocity = (linear_vel + angular_vel * Wheel.wheelSeperation / 2.0) / Wheel.wheelRadius
        right_velocity = (linear_vel - angular_vel * Wheel.wheelSeperation / 2.0) / Wheel.wheelRadius
        

        # Calculate new target ticks
        left_target_ticks = self.prev_l_ticks + (left_velocity * Wheel.ticksPerRadian)
        right_target_ticks = self.prev_r_ticks + (right_velocity * Wheel.ticksPerRadian)

        # Send the commands to the Arduino
        command = f"{left_target_ticks},{right_target_ticks}\n"
        # left_command = f"LEFT: {left_target_ticks}, {self.prev_l_ticks}"
        # right_command = f"RIGHT: {right_target_ticks}, {self.prev_r_ticks}"

        # self.get_logger().info(left_command)
        # self.get_logger().info(right_command)

        self.arduino.write(command)

        # if linear_vel == 0 and angular_vel == 0:
        #     left_target_ticks = self.prev_l_ticks
        #     right_target_ticks = self.prev_r_ticks
        #     self.arduino.write(f"{left_target_ticks},{right_target_ticks}\n")


        # Update previous wheel ticks
        # self.prev_l_ticks = left_target_ticks
        # self.prev_r_ticks = right_target_ticks


def main(args=None):
    rclpy.init(args=args)

    controller = MotionController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
