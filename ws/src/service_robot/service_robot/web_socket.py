#! /usr/bin/env python3 

# Libraries importing
from decouple import config
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces import Logger

# Utilities importing


# Environment variables definition


# Classes definition
class WebSocket(Node):
    """
    This class handle the websocket connection 
    aimed to communicate ROS2 with the web interface.
    """
    def __init__(self):
        super().__init__(node_name='websocket_topic')
        self._subscriber = self.create_subscription(String, 'websocket_topic', 
                                                    self.websocket_manager, 
                                                    10,
        )
        self._logger_publisher = self.create_publisher(Logger, 'logger_topic', 10)
    
    def websocket_manager(self, msg):
        """
        This function is responsible to handle the websocket connection.
        """
        log = Logger(log=f'Websocket data received: {msg.data}')
        self._logger_publisher.publish(log)

def main():
    rclpy.init()
    websocket = WebSocket()
    rclpy.spin(websocket)
    websocket.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()