#! /usr/bin/env python3 

# Libraries importing
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

# Utilities importing


# Environment variables definition


# Classes definition
class NavigationPlanner:
    """
    This class purpose is to handle the navigation system of the robot.\n
    Therefore, it will create a list of poses that robot needs to go through.
    """
    def __init__(self):
        self._nav = BasicNavigator()
        self._pose = PoseStamped()

    def create_pose(self, x, y, theta) -> bool:
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
        self._pose.header = Header(frame_id = 'map', 
                        stamp = self._nav.get_clock().now().to_msg()
                    )
        self._pose.pose = Pose(position = Point(x = x, y = y, z = theta), 
                        orientation = Quaternion(x = q_x, y = q_y, z = q_z, w= q_w)
                    )
        self._nav.waitUntilNav2Active()
        self._nav.goToPose(self._pose)
        if self._nav.isTaskComplete():
            self.create_pose(0.0, 0.0, 0.0)
            return True
        else:
            False

    def get_position(self):
        """
        Method used to get the current position of the robot.
        """
        pass
    
    # def path_setter(self, poses):
    #     """
    #     Method used to set the path of the robot.
    #     """
    #     self._nav.waitUntilNav2Active()
    #     self._nav.followWaypoints(poses)

def main():
    rclpy.init()
    navigation_planner = NavigationPlanner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
