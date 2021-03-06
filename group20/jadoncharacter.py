# This is necessary to find the main code
import sys
import queue
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class JadonCharacter(CharacterEntity):

    def do(self, wrld):
        path = self.astar(wrld)

        meloc = self.findme(wrld)

        #Find direction of movement
        dx = path[1][0] - meloc[0]
        dy = path[1][1] - meloc[1]
        print(dx)
        print(dy)

        self.move(dx, dy)

    def astar(self, wrld):
        start = self.findme(wrld)
        frontier = queue.PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        goal = self.getgoal(wrld)

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                path = [current]
                while came_from[current] != None:
                    path.insert(0, came_from[current])
                    current = came_from[current]
                return path

            for next in self.neighbors(current, wrld):
                new_cost = cost_so_far[current] + self.cost(current, next, wrld)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next, wrld)
                    frontier.put(next, priority)
                    came_from[next] = current




    def getgoal(self, wrld):
        for x in range (0, wrld.width()):
            for y in range (0, wrld.height()):
                if wrld.exit_at(x, y):
                    return x, y

    def heuristic(self, goal, next, wrld):
        return 0

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

    def findme(self, wrld):
        for x in range (0, wrld.width()):
            for y in range (0, wrld.height()):
                if wrld.characters_at(x, y):
                    return x, y
