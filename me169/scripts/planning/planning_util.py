import heapq
import itertools

import numpy as np


class TileStates:
    """
    Defines all possible tile states as enum values.
    """
    UNKNOWN   = 0
    WALL      = 1
    ONDECK    = 2
    PROCESSED = 3


class PriorityQueue:
    """
    Priority Queue implementation inspired by python3 heapq documentation: 
        https://docs.python.org/3/library/heapq.html
    """

    REMOVED = '<removed-task>'

    def __init__(self) -> None:
        self._data = []                      # Stores the internal heapq
        self.entry_finder = {}               # Maps element -> entry
        self.counter = itertools.count()     # Used for duplicate priorities

    def push(self, el, priority):
        """ Inserts an element into the priority queue """
        if el in self.entry_finder:
            self.remove(el)
        count = next(self.counter)
        entry = [priority, count, el]
        self.entry_finder[el] = entry
        heapq.heappush(self._data, entry)

    def remove(self, el):
        """ Marks an element as removed. Raise KeyError if not found. """
        entry = self.entry_finder.pop(el)
        entry[-1] = self.REMOVED

    def pop(self):
        """ Remove and return the lowest priority task. Raise KeyError if empty. """
        while self._data:
            priority, count, el = heapq.heappop(self._data)
            if el is not self.REMOVED:
                del self.entry_finder[el]
                return el
        raise KeyError('pop from an empty priority queue')

    def is_empty(self):
        """ Returns whether the priority queue is empty """
        return len(self.entry_finder) == 0
