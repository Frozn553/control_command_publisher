import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from rclpy.qos import QoSProfile
import math
import sys

class ControlCmdPublisher(Node):
    def __init__(self, speed, angular_velocity, wheelbase):
        super().__init__('control_cmd_publisher')
        
        qos_profile = QoSProfile(depth=10)
        
        self.publisher_ = self.create_publisher(AckermannControlCommand, 'control/command/control_cmd', qos_profile)
        
        self.timer = self.create_timer(1.0 / 20, self.publish_control_cmd)
        
        self.speed = speed
        self.angular_velocity = angular_velocity
        self.wheelbase = wheelbase  

    def calculate_steering_angle(self):
        if self.speed == 0:
            return 0.0
        return math.atan(self.angular_velocity * self.wheelbase / self.speed)

    def publish_control_cmd(self):
        control_cmd = AckermannControlCommand()
        control_cmd.longitudinal.speed = self.speed
        control_cmd.lateral.steering_tire_angle = self.calculate_steering_angle() * (180 / math.pi)

        self.publisher_.publish(control_cmd)
        self.get_logger().info(f'Published control command: speed={self.speed}, steering_angle={control_cmd.lateral.steering_tire_angle}')

    # TODO change velocity and angular during time

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Please specify a scenario: scenario1, scenario2, or scenario3")
        return

    scenario = sys.argv[1]
    wheelbase = 0.253  # 假設輪距為 2.5 米

    if scenario == "scenario1":
        speed = 1.0  # m/s
        angular_velocity = 0.0  # rad/s
    elif scenario == "scenario2":
        speed = 2.0  # m/s
        angular_velocity = 0.0  # rad/s
    elif scenario == "scenario3":
        speed = 1.0  # m/s
        angular_velocity = 55.0  # rad/s
    elif scenario == "scenario4":
        speed = 2.0  # m/s
        angular_velocity = 50.0  # rad/s
    else:
        print("Invalid scenario specified. Choose from scenario1, scenario2, or scenario3.")
        return

    control_cmd_publisher = ControlCmdPublisher(speed, angular_velocity, wheelbase)
    rclpy.spin(control_cmd_publisher)
    control_cmd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
