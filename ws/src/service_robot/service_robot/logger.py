#! /usr/bin/env python3 

# Libraries importing
import logging 
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import LogMessage

# Utilities importing


# Environment variables definition


# Classes definition
class Logger(Node):
    def __init__(self):
        super().__init__('logger_topic')
        self._subscriber = self.create_subscription(LogMessage, 'logger_topic', 
                                                    self.log_manager, 
                                                    10,
        )
        self._logger = logging.getLogger('Service Robot Logger')
        self.logger_setup()
        self._logger.info('Logger Initializer')

    def logger_setup(self):
        logging.basicConfig(filename='../assets/logs/out.log', 
                                        filemode='a', 
                                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                                        datefmt='%d-%b-%y %H:%M:%S',
                                        level=logging.INFO,
        )

    def log_manager(self, msg):
        msg = msg.log
        self._logger.info(msg=msg)

def main():
    rclpy.init()
    logger_node = Logger()
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()