from devices.actuator import Actuator;

def initial_position(controller) :
    position = [ 2048 - 512, 2048 + 512, 2048, 2048, 2048 + 512, 2048 - 512 ];
    center_position(controller);
    for i in Actuator.middle_index + Actuator.lower_index :
        controller.set_position(i, 270);

    # for i in Actuator.middle_right_index :
    #     controller.set_position(i, Actuator.position.center + 512);

    # for i in Actuator.middle_left_index :
    #     controller.set_position(i, Actuator.position.center - 512);

    # for i in Actuator.lower_right_index :
    #     controller.set_position(i, Actuator.position.center + 512);

    # for i in Actuator.lower_left_index :
    #     controller.set_position(i, Actuator.position.center - 512);

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
