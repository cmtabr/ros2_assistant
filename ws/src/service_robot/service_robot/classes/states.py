#! /usr/bin/env python3 

# Libraries importing
from enum import Enum

# Utilities importing


# Environment variables definition


# Classes definition
class States(Enum): 
    """
    This class is used to define the status of the robot.
    Based on four possible states:
    - STANDBY: the robot is waiting for a new task
    - ONGOING: the robot is performing a task
    - COMPLETED: the robot has completed the task
    - ABORTED: the robot has aborted the task
    """
    STANDBY = 0
    ONGOING = 1
    COMPLETED = 2
    ABORTED = 3