# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game

sys.path.insert(1, '../group20')

from Character1 import Character

# Create the game
g = Game.fromfile('map.txt')

g.add_character(Character("me", # name
                                     "C",  # avatar
                                     0, 0  # position
))

# Run!
# Use this if you want to proceed automatically
g.go(1)
