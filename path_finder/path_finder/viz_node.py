#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from path_finder_interfaces.msg import Int2DArray

class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")

        self.path_sub = self.create_subscription(Int2DArray, "/path", self.path_callback, 10)

    def path_callback(self, msg: Int2DArray):
        for i in range(msg.rows):
            row_string = ""
            for j in range(msg.cols - 1):
                row_string += str(msg.data[i*msg.cols + j]) + ", "
            row_string += str(msg.data[i*msg.cols + (msg.cols - 1)])
            self.get_logger().info(row_string)
        

def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()