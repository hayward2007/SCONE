from devices.actuator import Actuator;

def initial_position(controller) :
    for i in Actuator.index :
        controller.set_speed(i, 10);
        center_position();

def center_position(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 1);
        controller.set_position(i, Actuator.position.center);

def disable_torque(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 0);

def enable_torque(controller) :
    for i in Actuator.index :
        controller.set_torque(i, 1);

