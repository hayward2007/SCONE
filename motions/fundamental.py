from devices.actuator import *;

class Fundamental :
    upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
    middle_initial_position = 240;
    lower_initial_position = 255;

def initial_position(controller) :
    center_position(controller);

    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    
    for i in Actuator.middle_index + Actuator.lower_index :
        controller.set_position(i, Fundamental.middle_initial_position);

    for i in Actuator.lower_index :
        controller.set_position(i, Fundamental.lower_initial_position);

def center_position(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 1);
        controller.set_position(i, 180);

def stand_position(controller) :
    for i in Actuator.middle_index + Actuator.lower_index :
        controller.set_position(i, 270);

def disable_torque(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 0);

def enable_torque(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 1);