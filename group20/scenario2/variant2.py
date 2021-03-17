# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster

# TODO This is your code!

sys.path.insert(1, '../group20')
from Character2 import Character


# Create the game
# random.seed(123)
g = Game.fromfile('map.txt')
g.add_monster(StupidMonster("stupid", # name
                            "S",      # avatar
                            3, 9      # position
))

g.add_character(Character("me", # name
                              "C",  # avatar
                              0, 0,  # position
                              5
))

# Run!
g.go(1)
