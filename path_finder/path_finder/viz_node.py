#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from path_finder_interfaces.msg import Int2DArray
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")
        self.path_sub = self.create_subscription(
            Int2DArray,
            "/path",
            self.path_callback,
            10
        )
        
        self.fig, self.ax = plt.subplots()
        plt.ion()  # turn on interactive mode
        plt.show()
        # define the color map
        self.cmap = ListedColormap(['white', 'black', 'green'])

    def path_callback(self, msg: Int2DArray):
        self.get_logger().info('Received path update')
        # create a numpy array from the data
        grid = np.array(msg.data).reshape((msg.rows, msg.cols))
        
        # clear the previous plot and draw a new one
        self.ax.clear()
        self.ax.imshow(grid, aspect='equal', cmap=self.cmap, 
                       norm=plt.Normalize(vmin=0, vmax=2))
        # update the grid display without blocking
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
