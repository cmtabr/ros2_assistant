#! /usr/bin/env python3 

# Libraries importing
from queue import Queue

# Utilities importing


# Environment variables definition


# Classes definition
class EventsQueue:
    def __init__(self):
        """
        Initialize the queue with a maximum size of 5.
        """
        self._queue = Queue(maxsize=5)

    def put_event(self, event):
        """
        Method used to put an event in the queue.
        """
        self._queue.put(event)

    def get_event(self):
        """
        Method used to get an event from the queue.
        """
        return self._queue.get()
    
    def queue_size(self):
        """
        Method used to get the queue size.
        """
        return self._queue.qsize()
    
