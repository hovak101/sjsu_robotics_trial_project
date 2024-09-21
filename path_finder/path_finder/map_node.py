#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from path_finder_interfaces.msg import Int2DArray

class MapNode(Node):
    def __init__(self):
        try:
            super().__init__("map_node")
            self.get_logger().info("fuck you.")
            # Create the publisher
            self.map_pub = self.create_publisher(Int2DArray, "/map", 10)
            self.get_logger().info("Publisher created successfully.")

            # Initialize the String message
            self.my_map = Int2DArray()
            map_2d = [[0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0]]
            # flatten and convert to boolean
            self.my_map.data = [item for row in map_2d for item in row]
            self.my_map.rows = 5
            self.my_map.cols = 6
            self.get_logger().info("Map data initialized.")
            
            # Set up a timer to publish the message every 2 seconds
            self.timer = self.create_timer(2.0, self.publish_map)
            self.get_logger().info("Timer created.")
        except Exception as e:
            self.get_logger().error("Error during initialization: {}".format(e))

    def publish_map(self):
        # Publish the message and log it
        self.map_pub.publish(self.my_map)
        self.get_logger().info(f"Data: {self.my_map.data}, Rows: {self.my_map.rows}, Columns: {self.my_map.cols}")
        

def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)  # Set logging level to DEBUG
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
