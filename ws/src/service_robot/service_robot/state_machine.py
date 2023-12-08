#! /usr/bin/env python3 

# Libraries importing
from decouple import config
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import LogMessage

# Utilities importing
from .classes import EventsQueue, States


# Environment variables definition



# Classes definition
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_topic')
        self._subscriber = self.create_subscription(String, 'state_machine_topic', 
                                                    self.state_machine_callback, 
                                                    10,
        )
        self._logger_publisher = self.create_publisher(LogMessage, 'logger_topic', 10)
        self._robot_publisher = self.create_publisher(String, 
                                                    'robot_controller_topic', 10
        )
        # self._events_queue = EventsQueue()
        # self._states = States
        # self._current_state = self._states.STANDBY
    
    def state_machine_callback(self, msg):
        """
        This function is responsible to handle the websocket connection.
        """
        message = LogMessage(log=f'Websocket data received: {msg.data}')
        self._logger_publisher.publish(message)
        self._robot_publisher.publish(msg)

    # def process_queue(self):
    #     """
    #     This function is responsible to process the queue.
    #     """
    #     if self._events_queue.queue_size() > 0:
    #         self._current_state = self._states.MOVING
    #         self._logger_publisher.publish(f'Current state: {self._current_state}')
    #         self._robot_publisher.publish(self._events_queue.get_event())
    #         self._current_state = self._states.STANDBY
    #         self._logger_publisher.publish(f'Current state: {self._current_state}')

def main():
    rclpy.init()
    state_machine = StateMachine()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
