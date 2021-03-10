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
        self.max_depth = max_depth

    def do(self, wrld):
        dist_m = float('inf')

        try:
            m = next(iter(wrld.monsters.values()))[0]
            dist_m = math.sqrt((m.x - self.x)**2 + (m.y - self.y)**2)
        except:
            pass



        # calc distance from character to monster
        if dist_m > 3:
            # far enough away to just use A*
            path = self.astar(wrld)

            meloc = next(iter(wrld.characters.values()))[0]

            #Find direction of movement
            dx = path[1][0] - meloc.x
            dy = path[1][1] - meloc.y

            self.move(dx, dy)
        else:
            # use expectimax to aviod monster and make some progress
            dx, dy, bomb = self.expectimax(wrld)

            self.move(dx, dy)

    # Do expectimax search
    # PARAM [World] wrld The initial World object to start the expectimax with
    # RETURN [int, int] x and y values for the action to take
    def expectimax(self, wrld):

        c = next(iter(wrld.characters.values()))[0]

        xmax = 0
        ymax = 0
        bomb = 0
        max = float('-inf')

        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x+dx >= 0) and (c.x+dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the character is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (c.y+dy >=0) and (c.y+dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(c.x+dx, c.y+dy):# and not wrld.explosion_at(m.x+dx, m.y+dy):
                                # Set move in wrld
                                c.move(dx, dy)

                                # evaluate placing a bomb
                                for i in range(2):
                                    if i == 1:
                                        c.place_bomb()

                                    newwrld, events = wrld.next()

                                    val = self.exp_value(newwrld, 1)

                                    if val > max:
                                        max = val
                                        xmax = dx
                                        ymax = dy
                                        bomb = i

        return xmax, ymax, bomb

    # Calculate chance node for expectimax
    # PARAM [World] wrld The World object to calculate the chance node on
    # PARAM [list] events List of events that happend in the world
    # PARAM [int] depth Current depth of the expectimax
    # PARAM [(int, int)] old_action Action that was taken to get to this world state
    # RETURN [int, (int, int)] A score for the world and an action to take
    #      Chance node does not choose actions. Always return old_action for the action.                   
    def exp_value(self, wrld, depth):

        if self.game_over(wrld.events) or depth == self.max_depth:
            return self.eval_state(wrld, depth)

        v = 0

        m = next(iter(wrld.monsters.values()))[0]
        #
        # Go through the possible 8-moves of the monster
        #
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (m.x+dx >=0) and (m.x+dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (m.y+dy >=0) and (m.y+dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(m.x+dx, m.y+dy):
                                # Set move in wrld
                                m.move(dx, dy)
                                # Get new world
                                (newwrld, events) = wrld.next()

                                # find probability of actions taken
                                p = self.get_probability(newwrld)
                                
                                # eval max node
                                res = self.max_value(newwrld, depth+1)
                                v = v + (p * res)

        return v
    
    # Calculate max node for expectimax
    # PARAM [World] wrld The World object to calculate the max node on
    # PARAM [list] events List of events that happend in the world
    # PARAM [int] depth Current depth of the expectimax
    # PARAM [(int, int)] old_action Action that was taken to get to this world state
    # RETURN [int, (int, int)] A score for the world and an action to take
    def max_value(self, wrld, depth):

        if self.game_over(wrld.events) or depth == self.max_depth:
            return self.eval_state(wrld, depth)

        v = float('-inf')

        # get first character on map
        m = next(iter(wrld.characters.values()))[0]

        #
        # Go through the possible 8-moves of the character
        #
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (m.x+dx >=0) and (m.x+dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the character is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (m.y+dy >=0) and (m.y+dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(m.x+dx, m.y+dy):
                                # Set move in wrld
                                m.move(dx, dy)
                                # Get new world
                                (newwrld, events) = wrld.next()
                                
                                # eval chance node
                                res = self.exp_value(newwrld, depth+1)

                                # expectimax max part
                                if res > v:
                                    v = res
                        
        return v

    # Get the probability of a world state.
    # PARAM [World] wrld The world to calculate the probability for.
    # RETURN [float] The probability of getting to this world state.
    def get_probability(self, wrld):
        # for now assume stupid monster all actions equaly likely to happen
        m = next(iter(wrld.monsters.values()))[0]

        # count number of possible moves around stupid monster
        count = 0
        for dx in [-1, 0, 1]:
                # Avoid out-of-bound indexing
                if (m.x+dx >=0) and (m.x+dx < wrld.width()):
                    # Loop through delta y
                    for dy in [-1, 0, 1]:
                        # Make sure the character is moving
                        if (dx != 0) or (dy != 0):
                            # Avoid out-of-bound indexing
                            if (m.y+dy >=0) and (m.y+dy < wrld.height()):
                                # No need to check impossible moves
                                if not wrld.wall_at(m.x+dx, m.y+dy):
                                    count += 1

        return (1/count)

    # Check if the game has ended
    # PARAM [list] events List of world events
    # RETURN [bool] True of the game ended. False otherwise.
    def game_over(self, events):

        for e in events:
            if (e.tpe == Event.BOMB_HIT_CHARACTER or 
                e.tpe == Event.CHARACTER_KILLED_BY_MONSTER or
                e.tpe == Event.CHARACTER_FOUND_EXIT):
                
                if e.character.name == self.name:
                        return True

        return False

    # Gives a world state a score.
    # PARAM [World] wrld World to calculate score for.
    # PARAM [list] events List of world events
    # PARAM [int] depth Current depth of expectimax search
    # RETURN [float] Score for this world state
    def eval_state(self, wrld, depth):

        for e in wrld.events:
            # character dead, bad state
            if (e.tpe == Event.BOMB_HIT_CHARACTER or 
                e.tpe == Event.CHARACTER_KILLED_BY_MONSTER):
                
                if e.character.name == self.name:
                        return -1000/depth

            # character escaped, good state
            if (e.tpe == Event.CHARACTER_FOUND_EXIT):
                if e.character.name == self.name:
                        return 1000/depth

        c = next(iter(wrld.characters.values()))[0]

        dist_m = 0
        try:
            m = next(iter(wrld.monsters.values()))[0]
            dist_m = math.sqrt((m.x - c.x)**2 + (m.y - c.y)**2)

        except Exception as ex:
            pass

        # basically greedy best first search to move closer to exit
        dist_e = math.sqrt((wrld.exitcell[0] - c.x)**2 + (wrld.exitcell[1] - c.y)**2)
        return -dist_e

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
