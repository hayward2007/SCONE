from motions.fundamental import *;

def set_drive_mode(controller) :
    set_drive_position(controller);
    for i in Actuator.lower_index :
        controller.set_mode(i, Actuator.mode.velocity_control);
        controller.set_torque(i, 1);

def set_drive_position(controller) :
    stand_position(controller);
    controller.set_position(Actuator.lower_right_index[1], 90);
    controller.set_position(Actuator.lower_left_index[1], 90);

def drive_forward(controller) :
    for i in Actuator.lower_index :
        controller.set_speed(i, 10, mode = Actuator.mode.velocity_control);

def drive_backward() :
    print("asdf");

def drive_left() :
    print("asdf");

def drive_right() :
    print("asdf");

def drive_stop(controller) :
    for i in Actuator.lower_index :
        controller.set_speed(i, 0, mode = Actuator.mode.velocity_control);
