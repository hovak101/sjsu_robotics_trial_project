#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class GoalAndStartNode(Node):
    def __init__(self):
        super().__init__("goal_and_start_node")

        self.goal_pub = self.create_publisher(Point, "/goal", 10)
        self.start_pub = self.create_publisher(Point, "/start", 10)

        self.get_logger().info('Node has started. Waiting for user input...')

        # timer is of blocking type; won't create a new instance 
        # until current instance returns
        self.timer = self.create_timer(0.01, self.prompt_for_coordinates)

    def prompt_for_coordinates(self):
        goal_x = int(input("Enter goal's row index: "))
        goal_y = int(input("Enter goal's col index: "))
        start_x = int(input("Enter start's row index: "))
        start_y = int(input("Enter start's col index: "))

        # create Point messages for start and goal
        # goal:
        goal_point = Point()
        goal_point.x = float(goal_x)
        goal_point.y = float(goal_y)
        goal_point.z = 0.0

        # start:
        start_point = Point()
        start_point.x = float(start_x)
        start_point.y = float(start_y)
        start_point.z = 0.0

        # publish the messages
        self.goal_pub.publish(goal_point)
        self.start_pub.publish(start_point)

        self.get_logger().info("Published new coordinates. Waiting for user input...")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = GoalAndStartNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()