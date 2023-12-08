#! /usr/bin/env python3 

# Libraries importing
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import LogMessage

# Utilities importing
from .classes import NavigationPlanner

# Environment variables definition


# Classes definition
class RobotController(Node):
    """
    This class is responsible for move the robot on websocket positions
    received from the API.
    """
    def __init__(self):
        super().__init__('robot_controller_topic')
        self._subscrber = self.create_subscription(String, 'robot_controller_topic', 
                                                    self.robot_controller_callback, 
                                                    10,
        )
        self._logger_publisher1 = self.create_publisher(LogMessage, 'logger_topic', 10)
        self._navigation_planner = NavigationPlanner()

    def robot_controller_callback(self, msg):
        """
        This function is responsible to handle the websocket connection.
        """
        msg = msg.data
        x, y = self.extract_coordinates(msg)
        log = LogMessage(log=f'Going to positions: {x}, {y}')
        self._logger_publisher1.publish(log)
        result = self._navigation_planner.create_pose(x, y, 0.0)
        if result:
            log = LogMessage(log=f'Arrived at positions: {x}, {y}')
            self._logger_publisher1.publish(log)
        else:
            pass

    def extract_coordinates(self, input_str):
        match = re.search(r'\[([\d.]+),\s*([\d.]+)\]', input_str)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            return x, y
        else:
            return None
    
def main():
    rclpy.init()
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()