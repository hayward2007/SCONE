from motions.drive import *;
from motions.fundamental import *;

def set_stair_mode(controller) :
    for i in Actuator.upper_index :
        controller.set_position(i, 180);
    for i in Actuator.middle_left_index :
        # controller.set_position(i, 210);
        controller.set_position(i, 270);
    for i in Actuator.middle_right_index :
        controller.set_position(i, 180);
    for i in Actuator.lower_index :
        # controller.set_position(i, 270);
        controller.set_position(i, 270);

def climb_stair(controller) :
    for i in Actuator.lower_index :
        controller.set_speed(i, 100);
    
    for i in Actuator.lower_index :
        controller.set_raw_position(i, 1048575);

    time.sleep(5);

    for i in Actuator.lower_index :
        controller.set_torque(i, 0);

    time.sleep(0.5);

    for i in Actuator.lower_index :
        controller.set_torque(i, 1);

    # disable_torque(controller);

    # for i in Actuator.lower_index :
        # controller.set_speed(i, 0);
