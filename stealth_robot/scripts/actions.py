#!/usr/bin/env python3

from move_robot import RobotMover
from pi import Pi


class StealthActions:
    """ Define actions for stealth robot """

    # assumed that the start position is facing to the right
    def __init__(self, distance=1.0):
        self.mover = RobotMover()
        self.pi_calc = Pi()
        self.PI = self.pi_calc.PI

        # distance value is arbitrary
        self.distance = distance

    def up(self):
        # print("move up")
        self.mover.move(self.distance)

    def down(self):
        # negative distance to move down
        self.mover.move(-1*self.distance)

    def left(self):
        # rotate 90 degrees anticlockwise and move forward
        self.mover.rotate(self.PI/2, clockwise=False)
        self.mover.move(self.distance)

    def right(self):
        # rotate 90 degrees clockwise and move forward
        self.mover.rotate(self.PI/2, clockwise=True)
        self.mover.move(self.distance)
