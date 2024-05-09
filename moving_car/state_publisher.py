import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('State Publisher Node started')

    def move_forward(self, duration=5.0, linear_velocity=0.2):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity

        start_time = time.time()
        while time.time() - start_time < duration:
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info('Moving forward...')
            time.sleep(0.1)


    def move_backward(self, duration=3.0, linear_velocity=-0.2):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity

        start_time = time.time()
        while time.time() - start_time < duration:
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info('Moving backward...')
            time.sleep(0.1)
    
    def destroy_node(self):
        self.get_logger().info('Destroying Node...')
        super().destroy_node()
    
    def move_round(self, duration=5.0, angular_velocity=0.5):
        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity

        start_time = time.time()
        while time.time() - start_time < duration:
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info('Moving round...')
            time.sleep(0.1)
    
    def move_random(self, duration=15.0):
        twist_msg = Twist()

        start_time = time.time()
        while time.time() - start_time < duration:
            twist_msg.linear.x = random.uniform(-0.5, 0.5)
            twist_msg.angular.z = random.uniform(-math.pi, math.pi)
            
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info('Moving random...')
            time.sleep(3)
        
    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.twist_publisher.publish(twist_msg)
        self.get_logger().info('Stopping...')
    

def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()

    # Move forward for 5 seconds
    state_publisher.move_forward()
    
    state_publisher.move_random()

    # # Move backward for 3 seconds
    # state_publisher.move_backward()
    
    # state_publisher.move_round()
    
    # # Move forward for 5 seconds more faster
    # state_publisher.move_forward(linear_velocity=0.5)
        
    state_publisher.stop()
    
    rclpy.spin(state_publisher)
    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
