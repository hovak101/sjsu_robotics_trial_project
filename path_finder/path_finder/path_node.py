#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from path_finder_interfaces.msg import Int2DArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from utils.list_node import ListNode

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
        self.map_sub = self.create_subscription(Int2DArray, "/map", 
                                                self.map_callback, 10)

        self.path_pub = self.create_publisher(Int2DArray, "/path", 10)

    def goal_and_start_callback(self, goal_msg: Point, start_msg: Point):
        self.get_logger().info(str(goal_msg))
        self.get_logger().info(str(start_msg))
        self.goal_point = (int(goal_msg.x), int(goal_msg.y))
        self.start_point = (int(start_msg.x), int(start_msg.y))
        map_with_path = self.find_path(self.my_map, self.start_point, 
                                       self.goal_point)
        self.path_pub.publish(map_with_path)
    
    def map_callback(self, msg: Int2DArray):
        self.my_map = msg

    def find_path(self, a_map: Int2DArray, start: Point, end: Point):
        # A* algorithm
        # convert a_map into 2D array for path finding
        self.get_logger().info(str(a_map))
        a_map_2d = []
        for i in range(a_map.rows):
            row_list = []
            for j in range(a_map.cols):
                row_list.append(a_map.data[i*a_map.cols + j])
            a_map_2d.append(row_list)

        # create start and end node
        start_node = ListNode(None, tuple(self.start_point))
        start_node.g = start_node.h = start_node.f = 0
        end_node = ListNode(None, tuple(self.goal_point))
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []

        open_list.append(start_node)

        # loop until no possible path
        while len(open_list) > 0:

            # current node has lowest cost
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # if goal is found
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                # convert to msg format
                for coordinate in path:
                    row = coordinate[0]
                    col = coordinate[1]
                    a_map.data[row*self.my_map.cols + col] = 2

                return a_map

            # generate children
            children = []
            # possible adjacent squares
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), 
                                 (-1, -1), (-1, 1), (1, -1), (1, 1)]:

                node_position = (current_node.position[0] + new_position[0], 
                                 current_node.position[1] + new_position[1])
                
                if (node_position[0] > a_map.rows or node_position[0] < 0 
                or node_position[1] > a_map.cols or node_position[1] < 0):
                    continue

                # make sure map is empty at this position
                if a_map_2d[node_position[0]][node_position[1]] != 0:
                    continue

                new_node = ListNode(current_node, node_position)
                children.append(new_node)

            for child in children:

                # if child is on the closed list, don't add to any list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # else find f, g, h costs
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((
                    child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # if child is already in the open list and cost is greater, ignore
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # otherwise add the child to the open list
                open_list.append(child)


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()