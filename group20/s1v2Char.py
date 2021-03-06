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
        m = next(iter(wrld.monsters.values()))[0]

        if math.sqrt((m.x - self.x)**2 + (m.y - self.y)**2) > 3:
            path = self.astar(wrld)

            meloc = next(iter(wrld.characters.values()))[0]

            #Find direction of movement
            dx = path[1][0] - meloc.x
            dy = path[1][1] - meloc.y

            self.move(dx, dy)
        else:
            (dx, dy) = self.expectimax(wrld)

            self.move(dx, dy)

    def expectimax(self, wrld):
        return self.max_value(wrld, list(), 1, None)[1]

    # just one monsert right now
    def exp_value(self, wrld, events, depth, old_action):
        if self.game_over(events) or depth == self.max_depth:
            return self.eval_state(wrld, events, depth), old_action

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
                                (newwrld,events) = wrld.next()

                                # find probability of actions taken
                                p = self.get_probability(wrld)
                                
                                # eval max node
                                res = self.max_value(newwrld, events, depth+1, (dx, dy))
                                v = v + (p * res[0])

        # don't know if we can pick an action here
        # maybe need to have to keep depths a certian value
        return v, old_action
            
    def max_value(self, wrld, events, depth, old_action):

        if self.game_over(events) or depth == self.max_depth:
            return self.eval_state(wrld, events, depth), old_action

        v = float('-inf')
        new_action = None

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
                                (newwrld,events) = wrld.next()
                                
                                # eval chance node
                                res = self.exp_value(newwrld, events, depth+1, (dx, dy))

                                # expectimax max part
                                if res[0] > v:
                                    v = res[0]
                                    new_action = (dx, dy)
                        
        return v, new_action

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

    def game_over(self, events):

        for e in events:
            if (e.tpe == Event.BOMB_HIT_CHARACTER or 
                e.tpe == Event.CHARACTER_KILLED_BY_MONSTER or
                e.tpe == Event.CHARACTER_FOUND_EXIT):
                
                if e.character.name == self.name:
                        return True

        return False

    def eval_state(self, wrld, events, depth):

        # print("wrld ",wrld)
        # print("wrld ",next(iter(wrld.characters.values())))

        for e in events:
            # character dead, bad state
            if (e.tpe == Event.BOMB_HIT_CHARACTER or 
                e.tpe == Event.CHARACTER_KILLED_BY_MONSTER):
                
                if e.character.name == self.name:
                        return -100

            # character escaped, good state
            if (e.tpe == Event.CHARACTER_FOUND_EXIT):
                if e.character.name == self.name:
                        return 100

        # return -len(self.astar(wrld))
        c = next(iter(wrld.characters.values()))[0]
        return -math.sqrt((wrld.exitcell[0] - c.x)**2 + (wrld.exitcell[1] - c.y)**2)

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
