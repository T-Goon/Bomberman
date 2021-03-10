# This is necessary to find the main code
import sys
import queue
import math
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class Character(CharacterEntity):

    def do(self, wrld):
        path = self.astar(wrld)

        meloc = next(iter(wrld.characters.values()))[0]

        #Find direction of movement
        dx = path[1][0] - meloc.x
        dy = path[1][1] - meloc.y
        print(dx)
        print(dy)

        self.move(dx, dy)

    def astar(self, wrld):
        c = next(iter(wrld.characters.values()))[0]
        start = (c.x, c.y)
        frontier = queue.PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        goal = wrld.exitcell

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                path = [current]
                while came_from[current] != None:
                    path.insert(0, came_from[current])
                    current = came_from[current]
                return path

            for nextm in self.neighbors(current, wrld):
                new_cost = cost_so_far[current] + self.cost(current, nextm, wrld)
                if nextm not in cost_so_far or new_cost < cost_so_far[nextm]:
                    cost_so_far[nextm] = new_cost
                    priority = new_cost + self.heuristic(goal, nextm, wrld)
                    frontier.put(nextm, priority)
                    came_from[nextm] = current

    def heuristic(self, goal, next, wrld):
        dx = abs(next[0] - goal[0])
        dy = abs(next[1] - goal[1])

        # Euclidean distance
        return math.sqrt(dx * dx + dy * dy)

    def neighbors(self, current, wrld):
        neighbors = []
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (current[0] + dx >= 0) and (current[0] + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure we are moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (current[1] + dy >= 0) and (current[1] + dy < wrld.height()):
                            neighbors.append((current[0] + dx, current[1] + dy))
        return neighbors


    def cost(self, current, next, wrld):
        if wrld.wall_at(next[0], next[1]):
            return float('inf')
        else:
            return 1

