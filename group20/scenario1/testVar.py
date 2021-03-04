# This is necessary to find the main code
import sys
import random
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game


# TODO This is your code!
sys.path.insert(1, '../group20')

# Uncomment this if you want the empty test character
from testcharacter import TestCharacter

random.seed(123)

# Uncomment this if you want the interactive character
# from interactivecharacter import InteractiveCharacter

# Create the game
g = Game.fromfile('map2.txt')

from monsters.stupid_monster import StupidMonster
g.add_monster(StupidMonster("stupid", # name
                            "S",      # avatar
                            0, 0      # position
))

# g.add_monster(StupidMonster("stupid2", # name
#                             "S",      # avatar
#                             7, 0      # position
# ))

# from monsters.selfpreserving_monster import SelfPreservingMonster
# g.add_monster(SelfPreservingMonster("aggressive", # name
#                                     "A",          # avatar
#                                     3, 13,        # position
#                                     2             # detection range
# ))

# TODO Add your character

# Uncomment this if you want the test character
g.add_character(TestCharacter("me", # name
                              "C",  # avatar
                              2, 2,  # position
                              5
))

# Uncomment this if you want the interactive character
# g.add_character(InteractiveCharacter("me", # name
#                                      "C",  # avatar
#                                      0, 0  # position
# ))

# Run!

# Use this if you want to press ENTER to continue at each step
# g.go(0)

# Use this if you want to proceed automatically
g.go(1)
