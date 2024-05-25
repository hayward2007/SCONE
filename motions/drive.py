import time;
from motions.fundamental import *;

def set_drive_mode(controller) :
    for i in Actuator.upper_index :
        controller.set_position(i, 180);
    for i in Actuator.lower_index :
        controller.set_position(i, 270);
        if i == 3 or i == 4 :
            controller.set_position(i, 90);

def drive_forward(controller) :
    for i in Actuator.lower_index :
        controller.set_raw_position(i, 1048575);

def drive_backward() :
    print("asdf");

def drive_left() :
    print("asdf");

def drive_right() :
    print("asdf");

def drive_stop(controller) :
    for i in Actuator.lower_index :
        controller.set_speed(i, 0, mode = Actuator.mode.velocity_control);
