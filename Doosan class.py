from DRCF import *
import math


class Doosan:
    def __init__(self):
        self.current_posj = get_current_posj()
        self.velocity = 300
        self.accelaration = 300
        self.railstation = posx(718.7, 406.8, 699.7, 18.0, 125.8, -30.4)
        # self.camerastation = posx[]
        # self.PCBmold = posx[]

    def current_position(self):
        coordinates = get_current_posx()[0]
        return [round(coordinate, 1) for coordinate in coordinates]

    def add(self, add):
        addto(self.current_posj, add)

    def move(self, position):
        movel(position, v = self.velocity, a = self.accelaration)

    def move_to_railstation(self):
        self.move(self.railstation)

        if self.current_position() == self.railstation:
            tp_log('In the right position')
        else:
            tp_log('Not in the right position')

robot = Doosan()
robot.move_to_railstation()
