from motions.fundamental import *;

def set_drive_mode(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 0);
        controller.set_mode(i, 4);
        controller.set_torque(i, 1);

    for i in Actuator.upper_index :
        controller.set_position(i, 180);
    for i in Actuator.lower_index :
        if i == 15 or i == 16 :
            controller.set_position(i, 90);
        else :
            controller.set_position(i, 270);

def drive_forward(controller) :
    for i in Actuator.lower_index :
        controller.set_speed(i, 200);
    
    for i in Actuator.lower_index :
        controller.set_raw_position(i, 1048575);

    time.sleep(2);

    for i in Actuator.lower_index :
        controller.set_speed(i, 0);

    for i in Actuator.index :
        controller.set_torque(i, 0);
        controller.set_mode(i, 3);
        controller.set_torque(i, 1);

def drive_backward() :
    print("asdf");

def drive_left() :
    print("asdf");

def drive_right() :
    print("asdf");

def drive_stop(controller) :
    for i in Actuator.lower_index :
        controller.set_speed(i, 0, mode = Actuator.mode.velocity_control);
