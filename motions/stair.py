from motions.fundamental import *;
from motions.drive import *;

def set_stair_mode(controller) :
    for i in Actuator.upper_index :
        controller.set_position(i, 180);
    for i in Actuator.middle_left_index :
        controller.set_position(i, 180);
    for i in Actuator.lower_index :
        controller.set_position(i, 270);

def climb_stair(controller) :
    drive_forward(controller);
