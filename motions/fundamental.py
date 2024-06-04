import time;

from devices.actuator import *;
from devices.controller import Controller;

class Fundamental :
    upper_initial_position = [ 135, 135, 180, 180, 225, 225 ];
    middle_initial_position = 240;
    lower_initial_position = 255;

class Status :
    INITIALIZING = 0;
    STANDBY = 1;
    WALKING = 2;
    DRIVING = 3;
    CLIMBING = 4;

def initial_position(controller: Controller) :
    enable_torque(controller);
    controller.set_all_speed(30);
    for i in Actuator.middle_index :
        controller.set_position(i, Fundamental.middle_initial_position);
    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    for i in Actuator.lower_index :
        controller.set_position(i, Fundamental.lower_initial_position);
    time.sleep(2);

def low_initial_position(controller: Controller) :
    enable_torque(controller);
    controller.set_all_speed(30);

def start_position(controller: Controller) :
    enable_torque(controller);
    controller.set_all_speed(50);
    for i in Actuator.middle_index :
        controller.set_position(i, Fundamental.middle_initial_position - 105);
    time.sleep(0.5);
    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    for i in Actuator.lower_index :
        controller.set_speed(i, 150);
        controller.set_position(i, Fundamental.lower_initial_position);
    time.sleep(0.5);
    controller.set_all_speed(50);
    for i in Actuator.middle_index + Actuator.lower_index :
        controller.set_position(i, Fundamental.middle_initial_position);
    time.sleep(1);

def end_position(controller: Controller) :
    controller.set_all_speed(45);
    for i in Actuator.middle_index :
        controller.set_position(i, 165);
    time.sleep(1.5);
    disable_torque(controller);

def center_position(controller: Controller) :
    for i in Actuator.index :
        controller.set_torque(i, 1);
        controller.set_position(i, 180);

def stand_position(controller: Controller) :
    for i in Actuator.middle_index + Actuator.lower_index :
        controller.set_position(i, 270);

def disable_torque(controller: Controller) :
    for i in Actuator.index :
        controller.set_torque(i, 0);

def enable_torque(controller: Controller) :
    for i in Actuator.index :
        controller.set_torque(i, 1);
