import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from rclpy.qos import QoSProfile
import math
import sys

class ControlCmdPublisher(Node):
    def __init__(self, speed_list, angular_velocity_list, time_list, wheelbase, publish_rate):
        super().__init__('control_cmd_publisher')
        
        qos_profile = QoSProfile(depth=10)
        
        self.publisher_ = self.create_publisher(AckermannControlCommand, 'control/command/control_cmd', qos_profile)
        
        self.publish_rate = publish_rate  
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_control_cmd)
        
        self.speed_list = speed_list
        self.angular_velocity_list = angular_velocity_list
        self.time_list = time_list
        self.wheelbase = wheelbase  
        self.counter = 0  
        self.current_index = 0

    def calculate_steering_angle(self, speed, angular_velocity):
        if speed == 0:
            return 0.0
        return math.atan(angular_velocity * self.wheelbase / speed) * ( 180 / math.pi )

    def publish_control_cmd(self):
        self.counter += 1  
        elapsed_time = self.counter / self.publish_rate  

    
        if self.current_index < len(self.time_list) - 1 and elapsed_time >= self.time_list[self.current_index + 1]:
            self.current_index += 1


        speed = self.speed_list[self.current_index]
        angular_velocity = self.angular_velocity_list[self.current_index]

        control_cmd = AckermannControlCommand()
        control_cmd.longitudinal.speed = speed
        control_cmd.lateral.steering_tire_angle = self.calculate_steering_angle(speed, angular_velocity) 

        self.publisher_.publish(control_cmd)
        self.get_logger().info(f'Published control command: speed={speed:.2f} m/s,  steering_angle={control_cmd.lateral.steering_tire_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("choose: scenario1, scenario2, or scenario3")
        return

    scenario = sys.argv[1]
    wheelbase = 0.35  # 0.35 m
    publish_rate = 20  # Hz

    if scenario == "scenario1":
        speed_list = [1.0, 0.0]  # m/s
        angular_velocity_list = [0.0, 0.0]  # rad/s
        time_list = [0, 7]
    elif scenario == "scenario2":
        speed_list = [2.0 ,0.0]  # m/s
        angular_velocity_list = [0.0, 0.0]  # rad/s
        time_list = [0, 7]
    elif scenario == "scenario3":
        speed_list = [1.0, 2.0, 0.0]  # m/s
        angular_velocity_list = [0.8727, 0.8727, 0]  # rad/s
        time_list = [0, 5, 10]  # s
    else:
        print("Please choose scenario1, scenario2, 或 scenario3。")
        return

    control_cmd_publisher = ControlCmdPublisher(speed_list, angular_velocity_list, time_list, wheelbase, publish_rate)
    rclpy.spin(control_cmd_publisher)
    control_cmd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
