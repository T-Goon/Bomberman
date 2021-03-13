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

        # Find path using a*
        path = self.astar(wrld)

        # Find direction of movement
        dx = path[1][0] - self.x
        dy = path[1][1] - self.y

        self.move(dx, dy)

    # Do a* search
    # PARAM [World] wrld The initial World object to start the a* with
    # RETURN [int, (int, int)] the path (list of (x,y) tuples) in order from source to destination
    def astar(self, wrld):
        c = next(iter(wrld.characters.values()))[0]  # find our character
        start = (c.x, c.y)
        frontier = queue.PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        goal = wrld.exitcell

        # While we still have stuff in prio. queue, grab one
        while not frontier.empty():
            current = frontier.get()

            # If we found our goal, find path from source to destination and return path
            if current == goal:
                path = [current]
                while came_from[current] != None:
                    path.insert(0, came_from[current])
                    current = came_from[current]
                return path

            # If we didn't find goal, look around at all neighbors and add unvisited nodes to frontier
            for nextmove in self.neighbors(current, wrld):
                new_cost = cost_so_far[current] + self.cost(nextmove, wrld)
                if nextmove not in cost_so_far or new_cost < cost_so_far[nextmove]:
                    cost_so_far[nextmove] = new_cost
                    priority = new_cost + self.heuristic(goal, nextmove)
                    frontier.put(nextmove, priority)
                    came_from[nextmove] = current

    # Heuristic for a* algorithm
    # PARAM [int, int] goal x, y coordinates of exit
    # PARAM [int, int] m x, y coordinates of the next move (the one we are calculating heuristic for)
    # RETURN [int] the heuristic cost of the move m to goal
    def heuristic(self, goal, m):

        dx = abs(m[0] - goal[0])  # delta x from next move to goal
        dy = abs(m[1] - goal[1])  # delta y from next move to goal

        # Euclidean distance
        return math.sqrt(dx * dx + dy * dy)

    # Cost function for a* algorithm
    # PARAM [int, int] nextm x, y coordinates of the next move (the one we are calculating cost for)
    # PARAM [World] wrld The initial World object to calculate cost with
    # RETURN [int] the real cost of the next move
    def cost(self, nextm, wrld):
        # All movement is the same cost (1) unless it is a wall (inf)
        if wrld.wall_at(nextm[0], nextm[1]):
            return 12  # bomb timer + explosion time
        else:
            return 1

    # Neighbor function for a* algorithm
    # PARAM [int, int] current x, y coordinates of current node location (not character location)
    # PARAM [World] wrld The initial World object to calculate cost with
    # RETURN [int, (int, int)] list of neighbor node coordinates
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
