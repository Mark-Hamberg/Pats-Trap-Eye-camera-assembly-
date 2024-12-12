from DRCF import *
import math


class Doosan:
    def __init__(self):
        self.current_posj = get_current_posj()
        self.velocity = 10
        self.accelaration = 10
        self.railstation = posx(718.7, 406.8, 699.7, 18.0, 125.8, -30.4)
        # self.camerastation = posx[]
        # self.PCBmold = posx[]

# Get the positions of x and j with current_posisition-(). This can be used outside this class
# Move the robot with move(MOVE_COMMAND, DESIRED_END_LOCATION)
# I/O can be used with IO_output(I/O_NUMMER, ON/OFF), or IO_outputs(BIT_LIST[(EXAMPLE) -1, 3, 4, -6]) (turn 1 and 6 off, and 3,4 on)

    def current_positionx(self):
        coordinates = get_current_posx()[0]
        return [round(coordinate, 1) for coordinate in coordinates]
    
    def current_positionj(self):
        coordinates = get_current_posj()[0]
        return [round(coordinate, 1) for coordinate in coordinates]

    def home_pos(self): self.move(movej, posj(0.0, -0.0, -0.0, 0.0, -0.0, -0.0))

    def add(self, add): addto(self.current_posj, add)

    def move(self, action, position): action(position, v = self.velocity, a = self.accelaration)

    def IO_output(self, nummer, state=OFF): set_digital_output(nummer, state)

    def IO_outputs(self, nummers): set_digital_output(nummers)

    def move_to_railstation(self):
        self.move(movel, self.railstation)

        if self.current_position() == self.railstation:
            tp_log('In the right position')
        else:
            tp_log('Not in the right position')

    def hello(self):
        print("help")


robot = Doosan()

#robot.home_pos()
#robot.move(movej, posj(45,45, -45 ,0,0,-45))
#robot.move_to_railstation()
