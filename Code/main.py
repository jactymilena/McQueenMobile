from sys import path
path.append(r'C:\projets\McQueenMobile\Code') # TODO make this a relative pat

import car
import constants as const

move_dist = 2
frame_rate = 2
front_axe = const.Y_AXE
direction = 1

c = car.Car()
c.test()
# c.init_simulation("car", "obstacles", front_axe, direction)
# c.simulate(move_dist, frame_rate)