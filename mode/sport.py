from devices.actuator import *;
from devices.controller import *;

class Sport :
    # initial position
    __upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
    __middle_initial_position = 240;
    __lower_initial_position = 255;

    # this speed is used when high torque is required
    __safety_speed = 50;
    # this speed is for walking
    __walking_speed = 100;

    def __init__(self, controller: Controller) :
        enable_torque(controller);
        controller.set_all_speed(self.__safety_speed);

        # set to safety leg height
        for i in Actuator.middle_index :
            controller.set_position(i, 135);
        time.sleep(0.5);

        for i in Actuator.upper_index :
            controller.set_position(i, self.__upper_initial_position[i - 1]);
        for i in Actuator.lower_index :
            controller.set_speed(i, 200);
            controller.set_position(i, self.__lower_initial_position);
        time.sleep(0.7);

        controller.set_all_speed(self.__safety_speed);
        for i in Actuator.middle_index :
            controller.set_position(i, self.__middle_initial_position);
        time.sleep(1);

        # set all speed to walking speed
        controller.set_all_speed(self.__walking_speed);
