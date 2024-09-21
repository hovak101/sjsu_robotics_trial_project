#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from path_finder_interfaces.msg import Int2DArray
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ListNode():
    # A node class for A* Pathfinding
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
class PathNode(Node):
    def __init__(self):
        super().__init__("path_node")
        self.goal_sub = Subscriber(self, Point, "/goal")
        self.start_sub = Subscriber(self, Point, "/start")

        self.ts = ApproximateTimeSynchronizer(
            [self.goal_sub, self.start_sub], 
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )
        self.ts.registerCallback(self.goal_and_start_callback)
        self.map_sub = self.create_subscription(Int2DArray, "/map", self.map_callback, 10)

        self.path_pub = self.create_publisher(Int2DArray, "/path", 10)

        # # testing path for visualization node
        # map_2d = [[0, 0, 0, 0, 0, 0],
        #           [0, 0, 2, 2, 0, 0],
        #           [0, 2, 1, 1, 0, 0],
        #           [0, 2, 0, 0, 0, 0],
        #           [2, 0, 0, 1, 0, 0]]
        # # flatten and convert to boolean
        # self.my_map = Int2DArray()
        # self.my_map.data = [item for row in map_2d for item in row]
        # self.my_map.rows = 5
        # self.my_map.cols = 6
        # self.get_logger().info("Map data initialized.")
        # self.path_pub.publish(self.my_map)

    def goal_and_start_callback(self, goal_msg: Point, start_msg: Point):
        self.get_logger().info(str(goal_msg))
        self.get_logger().info(str(start_msg))
        self.goal_point = (int(goal_msg.x), int(goal_msg.y))
        self.start_point = (int(start_msg.x), int(start_msg.y))
        map_with_path = self.find_path(self.my_map, self.start_point, self.goal_point)
        self.path_pub.publish(map_with_path)
    
    def map_callback(self, msg: Int2DArray):
        # self.get_logger().info(str(msg))
        self.my_map = msg

    def find_path(self, a_map: Int2DArray, start: Point, end: Point):
        # A* algorithm
        # Convert a_map into 2D array
        self.get_logger().info(str(a_map))
        a_map_2d = []
        for i in range(a_map.rows):
            row_list = []
            for j in range(a_map.cols):
                row_list.append(a_map.data[i*a_map.cols + j])
            a_map_2d.append(row_list)

        # Create start and end node
        start_node = ListNode(None, tuple(self.start_point))
        start_node.g = start_node.h = start_node.f = 0
        end_node = ListNode(None, tuple(self.goal_point))
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                # CONVERT TO MSG
                for coordinate in path:
                    row = coordinate[0]
                    col = coordinate[1]
                    a_map.data[row*self.my_map.cols + col] = 2

                return a_map

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(a_map_2d) - 1) or node_position[0] < 0 or node_position[1] > (len(a_map_2d[len(a_map_2d)-1]) -1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if a_map_2d[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = ListNode(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()