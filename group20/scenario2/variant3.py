# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster


sys.path.insert(1, '../group20')
from Character2 import Character

# Create the game
# random.seed(123)
g = Game.fromfile('map.txt')
g.add_monster(SelfPreservingMonster("selfpreserving", # name
                                    "S",              # avatar
                                    3, 9,             # position
                                    1                 # detection range
))

g.add_character(Character("me", # name
                              "C",  # avatar
                              0, 0,  # position
                              5
))

# Run!
g.go(1)
