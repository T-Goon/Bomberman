# This is necessary to find the main code

import math
import sys
import queue

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from colorama import Fore, Back

from events import Event


class Character(CharacterEntity):

    def __init__(self, name, avatar, x, y, max_depth):
        super().__init__(name, avatar, x, y)

        # max depth of the minimax search
        self.max_depth = max_depth

    def do(self, wrld):

        # Calculate distance to the monster
        try:
            m = next(iter(wrld.monsters.values()))[0]

            dist_m = math.sqrt((m.x - self.x) ** 2 + (m.y - self.y) ** 2)
        except Exception as e:
            dist_m = float('inf')

        b = wrld.bombs.values()
        ex = wrld.explosions.values()

        # use minimax to aviod monster, death, and make some progress
        if (dist_m < 5 or len(b) > 0 or len(ex) > 0):
            dx, dy = self.minimax(wrld)

            self.move(dx, dy)

            # place a bomb if a monster is in range
            if dist_m < 5:
                self.place_bomb()
        else:
            # Find path using a*
            path = self.astar(wrld)

            # Find direction of movement
            dx = path[1][0] - self.x
            dy = path[1][1] - self.y

            # Find first wall in path
            wall = None
            for i in range(1, len(path)):
                if wrld.wall_at(path[i][0], path[i][1]):
                    wall = (path[i][0], path[i][1])
                    break

            self.move(dx, dy)

            # wall in your path is in range of a bomb
            try:
                if (math.sqrt((wall[0] - self.x) ** 2 + (wall[1] - self.y) ** 2) <= 4 and
                        (wall[0] == self.x or wall[1] == self.y)):
                    self.place_bomb()
            except:
                pass

            # place a bomb if there is a wall in your path
            if (wrld.wall_at(self.x + dx, self.y + dy)):
                self.place_bomb()

    # Do minimax search
    # PARAM [World] wrld The initial World object to start the minimax with
    # RETURN [int, (int, int)] The score and x and y values for the action to take
    def minimax(self, wrld):

        c = next(iter(wrld.characters.values()))[0]

        xmax = 0
        ymax = 0
        max = float('-inf')

        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx >= 0) and (c.x + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the character is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (c.y + dy >= 0) and (c.y + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(c.x + dx, c.y + dy):  # and not wrld.explosion_at(m.x+dx, m.y+dy):
                                # Set move in wrld
                                c.move(dx, dy)

                                # evaluate placing a bomb
                                for i in range(2):
                                    if i == 1:
                                        c.place_bomb()

                                    newwrld, events = wrld.next()

                                    # Calculate min node
                                    val = self.min_value(newwrld, float('-inf'), float('inf'), 1)

                                    # argmax
                                    if val > max:
                                        max = val
                                        xmax = dx
                                        ymax = dy

        return xmax, ymax

    # Return the max value of a world state and the action to get to that world state.
    # PARAM [World] board: the current world state
    # PARAM [int] a: alpha value
    # PARAM [int] b: beta value
    # PARAM [int] depth: max depth of the search
    # RETURN [int]: max score of the min nodes
    def max_value(self, wrld, a, b, depth):
        """The maxvalue function for alpha beta search algorithm."""

        # Game is over or search is at max depth
        if self.game_over(wrld.events) or depth == self.max_depth:
            return self.eval_stateMM(wrld, depth)

        v = float('-inf')

        # get first character on map
        c = next(iter(wrld.characters.values()))[0]

        #
        # Go through the possible 8-moves of the character
        #
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx >= 0) and (c.x + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the character is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (c.y + dy >= 0) and (c.y + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(c.x + dx, c.y + dy):  # and not wrld.explosion_at(m.x+dx, m.y+dy):
                                # Set move in wrld
                                c.move(dx, dy)

                                # Get new world
                                (newwrld, events) = wrld.next()

                                # alpha beta pruning logic
                                min = self.min_value(newwrld, a, b, depth + 1)

                                if min > v:
                                    v = min

                                if v >= b:  # prune
                                    return v

                                a = max(a, v)

        return v

    # Return the min value of a world state and the action to get to that state.
    # PARAM [board.Board] board: the current world state
    # PARAM [int] a: alpha value
    # PARAM [int] b: beta value
    # PARAM [int] depth: max depth of the search
    # RETURN [int]: min value of the world state
    def min_value(self, wrld, a, b, depth):
        """The minvalue function for alpha beta search algorithm."""

        # Game over or hit max depth
        if self.game_over(wrld.events) or depth == self.max_depth:
            return self.eval_stateMM(wrld, depth)

        v = float('inf')

        m = None
        try:
            m = next(iter(wrld.monsters.values()))[0]
        except:
            # there is no monster
            max = self.max_value(wrld, a, b, depth + 1)

            return max

        #
        # Go through the possible 8-moves of the monster
        #
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (m.x + dx >= 0) and (m.x + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (m.y + dy >= 0) and (m.y + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(m.x + dx, m.y + dy):
                                # Set move in wrld
                                m.move(dx, dy)
                                # Get new world
                                (newwrld, events) = wrld.next()

                                # alpha beta pruning logic
                                max = self.max_value(newwrld, a, b, depth + 1)

                                if max < v:
                                    v = max

                                if v <= a:  # prune
                                    return v

                                b = min(b, v)

        return v

    # Gives a world state a score.
    # PARAM [World] wrld World to calculate score for.
    # PARAM [int] depth Current depth of expectimax search
    # RETURN [float] Score for this world state
    def eval_stateMM(self, wrld, depth):

        for e in wrld.events:
            # character dead, bad state
            if (e.tpe == Event.BOMB_HIT_CHARACTER or
                    e.tpe == Event.CHARACTER_KILLED_BY_MONSTER):
                return -1000 / depth

            # character escaped, good state
            if (e.tpe == Event.CHARACTER_FOUND_EXIT):
                return 1000 / depth

        c = next(iter(wrld.characters.values()))[0]

        # get distance to monster
        dist_m = 0
        try:
            m = next(iter(wrld.monsters.values()))[0]
            dist_m = math.sqrt((m.x - c.x) ** 2 + (m.y - c.y) ** 2)

        except Exception as ex:
            pass

        # dist_e = math.sqrt((wrld.exitcell[0] - c.x)**2 + (wrld.exitcell[1] - c.y)**2)

        return dist_m

    # Check if the game has ended
    # PARAM [list] events List of world events
    # RETURN [bool] True of the game ended. False otherwise.
    def game_over(self, events):

        for e in events:
            if (e.tpe == Event.BOMB_HIT_CHARACTER or
                    e.tpe == Event.CHARACTER_KILLED_BY_MONSTER or
                    e.tpe == Event.CHARACTER_FOUND_EXIT):
                return True

        return False

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
        if wrld.wall_at(nextm[0], nextm[1]):
            return 12  # bomb timer + explosion time
        elif wrld.explosion_at(nextm[0], nextm[1]):
            return float('inf')  # don't kill yourself

        # Do not stand in range of a bomb that is exploding in one turn
        # Shouldn't ever do anything
        # try:
        #     b = next(iter(wrld.bombs.values()))[0]

        #     dist_b = math.sqrt((b.x - n[0])**2 + (b.y - n[1])**2)

        #     if(b.timer == 0 and
        #         (b.x == n[0] or b.y == n[1]) and
        #         dist_b <= 4):
        #         return float('inf')
        # except:
        #     pass

        # Don't path near monster
        try:
            m = next(iter(wrld.monsters.values()))[0]
            if math.sqrt((nextm[0] - m.x) ** 2 + (nextm[1] - m.y) ** 2) < 5:
                return float('inf')
        except:
            pass

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