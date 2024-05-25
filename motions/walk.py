import time
from devices.actuator import Actuator
from motions.fundamental import Fundamental;

def set_walk_position() :
    print("asdf");

def hold_dignoal_right_index(controller) :
    for i in Actuator.middle_diagonal_right_index :
        controller.set_position(i, 225);
        controller.set_position(i + 6, 240);

def hold_dignoal_left_index(controller) :
    for i in Actuator.middle_diagonal_left_index :
        controller.set_position(i, 225);
        controller.set_position(i + 6, 240);

def release_dignoal_right_index(controller) :
    for i in Actuator.middle_diagonal_right_index :
        controller.set_position(i, 240);
        controller.set_position(i + 6, 255);

def release_dignoal_left_index(controller) :
    for i in Actuator.middle_diagonal_left_index :
        controller.set_position(i, 240);
        controller.set_position(i + 6, 255);

def walk_forward() :
    print("asdf");

def walk_backward() :
    print("asdf");

def walk_left() :
    print("asdf");

def walk_right() :
    print("asdf");

def turn_right(controller) :
    hold_dignoal_right_index(controller);
    time.sleep(0.2);
    
    for i in Actuator.upper_diagonal_right_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] - 30);
    for i in Actuator.upper_diagonal_left_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] + 30);
    time.sleep(0.5);

    release_dignoal_right_index(controller);
    hold_dignoal_left_index(controller);
    time.sleep(0.2);

    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    time.sleep(0.5);

    release_dignoal_left_index(controller);


def turn_left(controller) :
    hold_dignoal_left_index(controller);
    time.sleep(0.5);
    
    for i in Actuator.upper_diagonal_left_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] - 30);
    for i in Actuator.upper_diagonal_right_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] + 30);
    time.sleep(0.5);

    release_dignoal_left_index(controller);
    hold_dignoal_right_index(controller);
    time.sleep(0.5);

    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    time.sleep(0.5);

    release_dignoal_right_index(controller);
