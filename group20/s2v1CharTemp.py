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
        # dist_e = math.sqrt((wrld.exitcell[0] - self.x)**2 + (wrld.exitcell[1] - self.y)**2)

        try:
            m = next(iter(wrld.monsters.values()))[0]

            dist_m = math.sqrt((m.x - self.x)**2 + (m.y - self.y)**2)
        except Exception as e:
            print("do ", e)
            dist_m = float('inf')
        

        b = wrld.bombs.values()
        ex = wrld.explosions.values()

        # # calc distance from character to monster
        # if (dist_m  > 3 or dist_e < 3) and len(b) == 0:
        #     # far enough away to just use A*
        #     path = self.astar(wrld)

        #     meloc = next(iter(wrld.characters.values()))[0]

        #     #Find direction of movement
        #     dx = path[1][0] - meloc.x
        #     dy = path[1][1] - meloc.y

        #     self.move(dx, dy)

        #     if(wrld.wall_at(self.x + dx, self.y + dy)):
        #         self.place_bomb()
                
        # else:
        #     # use minimax to aviod monster and make some progress
        #     score, (dx, dy), bomb = self.minimax(wrld)

        #     self.move(dx, dy)

        #     if bomb == 1:
        #         self.place_bomb()

        # use minimax to aviod monster and make some progress
        if(dist_m < 3 or len(b) > 0 or len(ex) > 0):
            dx, dy, bomb = self.minimax(wrld)

            self.move(dx, dy)

            if bomb == 1:
                self.place_bomb()
        else:
            # Find path using a*
            path = self.astar(wrld)

            # Find location of our character
            meloc = next(iter(wrld.characters.values()))[0]

            # Find direction of movement
            dx = path[1][0] - meloc.x
            dy = path[1][1] - meloc.y

            self.move(dx, dy)

            if(wrld.wall_at(self.x + dx, self.y + dy)):
                self.place_bomb()

    # Do minimax search
    # PARAM [World] wrld The initial World object to start the minimax with
    # RETURN [int, (int, int), int] The score and x and y values for the action to take
    #                                and if a bomb should be placed or not
    # Do minimax search
    # PARAM [World] wrld The initial World object to start the minimax with
    # RETURN [int, (int, int), int] The score and x and y values for the action to take
    #                                and if a bomb should be placed or not
    def minimax(self, wrld):

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

                                    val = self.min_value(newwrld, float('-inf'), float('inf'), 1)

                                    if val > max:
                                        max = val
                                        xmax = dx
                                        ymax = dy
                                        bomb = i

        return xmax, ymax, bomb

     # Return the max value of a world state and the action to get to that world state.
    # PARAM [World] board: the current world state
    # PARAM [int] a: alpha value
    # PARAM [int] b: beta value
    # PARAM [int] old_action: action taken to get to this board
    # PARAM [int] depth: max depth of the search
    # RETURN [int, (int, int)]: max value of the board state and the action to get to it
    def max_value(self, wrld, a, b, depth):
        """The maxvalue function for alpha beta search algorithm."""

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
                                for i in range(1):
                                    if i == 1:
                                        c.place_bomb()
                                    
                                    # Get new world
                                    (newwrld, events) = wrld.next()

                                    # alpha beta pruning logic
                                    min = self.min_value(newwrld, a, b, depth+1)

                                    if min > v:
                                        v = min

                                    if v >= b: # prune
                                        return v
                                    
                                    a = max(a, v)

        return v

    # Return the min value of a world state and the action to get to that state.
    # PARAM [board.Board] board: the current world state
    # PARAM [int] a: alpha value
    # PARAM [int] b: beta value
    # PARAM [int] old_action: action taken to get to this world
    # PARAM [int] depth: max depth of the search
    # RETURN [int, int]: min value of the world state and the action to get to it
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
            max = self.max_value(wrld, a, b, depth+1)

            return max
            
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

                                # alpha beta pruning logic
                                max = self.max_value(newwrld, a, b, depth+1)

                                if max < v:
                                    v = max

                                if v <= a: # prune
                                    return v
                                
                                b = min(b, v)

        return v

    # Gives a world state a score.
    # PARAM [World] wrld World to calculate score for.
    # PARAM [int] depth Current depth of expectimax search
    # RETURN [float] Score for this world state
    def eval_stateMM(self, wrld, depth):
        # ws = 0

        for e in wrld.events:
            # character dead, bad state
            if (e.tpe == Event.BOMB_HIT_CHARACTER or 
                e.tpe == Event.CHARACTER_KILLED_BY_MONSTER):
                    return -1000/depth

        #     if (e.tpe == Event.BOMB_HIT_WALL): 
        #             ws = 10

            # character escaped, good state
            if (e.tpe == Event.CHARACTER_FOUND_EXIT or
                e.tpe == Event.BOMB_HIT_MONSTER):
                    return 1000/depth

                    # # try:
                    # #     c = next(iter(wrld.characters.values()))[0]
                    # #     dist_e = math.sqrt((wrld.exitcell[0] - c.x)**2 + (wrld.exitcell[1] - c.y)**2)
                    # # except:
                    # #     # player exited
                    # #     pass

                    # return (1000/depth)

        c = next(iter(wrld.characters.values()))[0]

        dist_m = 0
        try:
            m = next(iter(wrld.monsters.values()))[0]
            dist_m = math.sqrt((m.x - c.x)**2 + (m.y - c.y)**2)

        except Exception as ex:
            pass

        bs = 0

        try:
            b = next(iter(wrld.bombs.values()))[0]
            bs = 1000000
        except:
            pass

        # try:
        #     m = next(iter(wrld.monsters.values()))[0]

        #     # monster in range of bomb good
        #     try:
        #         b = next(iter(wrld.bombs.values()))[0]

        #         dist_b = math.sqrt((m.x - b.x)**2 + (m.y - b.y)**2)
                
        #         # monster in range of bomb
        #         if (b.y == m.y or b.x == m.x) and dist_b <= 4:
        #             bs = 10000/depth
        #     except:
        #         pass
        # except:
        #     pass

        dist_e = math.sqrt((wrld.exitcell[0] - c.x)**2 + (wrld.exitcell[1] - c.y)**2)

        return -dist_e + dist_m + bs

        # except:
        #     # monsters all dead
        #     return -dist_e

        # dist_m = math.sqrt((m.x - c.x)**2 + (m.y - c.y)**2)

        # print("dis exit ", dist_e)

        # # Being close to monster same as death
        # if dist_m < 4:
        #     return -1000/depth

        # basically greedy best first search to move closer to exit
        # and away from monster
        return -dist_e

    # Do expectimax search
    # PARAM [World] wrld The initial World object to start the expectimax with
    # RETURN [int, int] x and y values for the action to take
    def expectimax(self, wrld):
        return self.max_valueE(wrld, list(), 1, None)

    # Calculate chance node for expectimax
    # PARAM [World] wrld The World object to calculate the chance node on
    # PARAM [list] events List of events that happend in the world
    # PARAM [int] depth Current depth of the expectimax
    # PARAM [(int, int)] old_action Action that was taken to get to this world state
    # RETURN [int, (int, int)] A score for the world and an action to take
    #      Chance node does not choose actions. Always return old_action for the action.                   
    def exp_value(self, wrld, events, depth, old_action):

        if self.game_over(events) or depth == self.max_depth:
            return self.eval_state(wrld, events, depth), old_action

        v = 0

        m = None
        try:
            m = next(iter(wrld.monsters.values()))[0]
        except:
            score, (dx, dy) = self.max_value(wrld, events, depth+1, old_action)

            return score, (dx, dy)
            
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
                                p = self.get_probability(newwrld, wrld)
                                
                                # eval max node
                                res = self.max_value(newwrld, events, depth+1, (dx, dy))
                                v = v + (p * res[0])

        return v, old_action
    
    # Calculate max node for expectimax
    # PARAM [World] wrld The World object to calculate the max node on
    # PARAM [list] events List of events that happend in the world
    # PARAM [int] depth Current depth of the expectimax
    # PARAM [(int, int)] old_action Action that was taken to get to this world state
    # RETURN [int, (int, int)] A score for the world and an action to take
    def max_valueE(self, wrld, events, depth, old_action):

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
                            if not wrld.wall_at(m.x+dx, m.y+dy) and not wrld.explosion_at(m.x+dx, m.y+dy):
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

    # Get the probability of a world state.
    # PARAM [World] wrld The world to calculate the probability for.
    # PARAM []
    # RETURN [float] The probability of getting to this world state.
    def get_probability(self, wrld, old_wrld):
        # agro monster, action of moving toward player is p(1) if
        # distance is < 3

        for e in wrld.events:
            if (e.tpe == Event.CHARACTER_KILLED_BY_MONSTER or
            e.tpe == Event.BOMB_HIT_CHARACTER or
            e.tpe == Event.BOMB_HIT_MONSTER):
                return 1

        m_old = next(iter(old_wrld.monsters.values()))[0]

        m = next(iter(wrld.monsters.values()))[0]
        try:
            c = next(iter(wrld.characters.values()))[0]
        except:
            return 1

        dist = math.sqrt((m.x - c.x)**2 + (m.y - c.y)**2)
        dist_old = math.sqrt((m_old.x - c.x)**2 + (m_old.y - c.y)**2)

        # close enough for monster to chase
        if dist < 3:
            # monster will always move toward player
            if dist < dist_old:
                return 1
            else:
                # shouldn't ever get here
                return 0
        else:
            # character not in range, moves are random

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
                
                    return True

        return False

    # Gives a world state a score.
    # PARAM [World] wrld World to calculate score for.
    # PARAM [list] events List of world events
    # PARAM [int] depth Current depth of expectimax search
    # RETURN [float] Score for this world state
    def eval_stateEM(self, wrld, events, depth):

        for e in events:
            # character dead, bad state
            if (e.tpe == Event.BOMB_HIT_CHARACTER or 
                e.tpe == Event.CHARACTER_KILLED_BY_MONSTER):
                    return -1000/depth

            # character escaped, good state
            if (e.tpe == Event.CHARACTER_FOUND_EXIT):
                    return 1000/depth

        c = next(iter(wrld.characters.values()))[0]
        dist_e = math.sqrt((wrld.exitcell[0] - c.x)**2 + (wrld.exitcell[1] - c.y)**2)

        # monsters all dead
        try:
            m = next(iter(wrld.monsters.values()))[0]
        except:
            return -dist_e

        # Being close to monster same as death
        if math.sqrt((m.x - c.x)**2 + (m.y - c.y)**2) < 4:
            return -1000/depth

        # basically greedy best first search to move closer to exit
        
        return -dist_e

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
            return 12
        elif wrld.explosion_at(nextm[0], nextm[1]):
            return float('inf')

        try:
            b = next(iter(wrld.bombs.values()))[0]

            dist_b = math.sqrt((b.x - nextm[0])**2 + (b.y - nextm[1])**2)

            if(b.timer == 0 and 
                (b.x == nextm[0] or b.y == nextm[1]) and
                dist_b <= 4):
                return float('inf')
        except:
            pass

        try:
            m = next(iter(wrld.monsters.values()))[0]
            if math.sqrt((nextm[0] - m.x)**2 + (nextm[1] - m.y)**2) < 4.3:
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
