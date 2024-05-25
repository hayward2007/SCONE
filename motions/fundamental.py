from devices.actuator import Actuator;

def initial_position(controller) :
    position = [ 135, 135, 180, 180, 225, 225 ];
    center_position(controller);

    for i in Actuator.upper_index :
        controller.set_position(i, position[i - 1]);
    
    for i in Actuator.middle_index + Actuator.lower_index :
        controller.set_position(i, 240);

    for i in Actuator.lower_index :
        controller.set_position(i, 255);

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
