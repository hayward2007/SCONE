import time;

# from main import status;
from devices.actuator import *;
from motions.fundamental import *;

WALK_STOP = False;

def set_walk_mode(controller) :
    initial_position(controller);

def hold_dignoal_right_index(controller) :
    for i in Actuator.middle_diagonal_right_index :
        controller.set_position(i, Fundamental.middle_initial_position - 15);
        controller.set_position(i + 6, Fundamental.lower_initial_position - 15);

def hold_dignoal_left_index(controller) :
    for i in Actuator.middle_diagonal_left_index :
        controller.set_position(i, Fundamental.middle_initial_position - 15);
        controller.set_position(i + 6, Fundamental.lower_initial_position - 15);

def release_dignoal_right_index(controller) :
    for i in Actuator.middle_diagonal_right_index :
        controller.set_position(i, Fundamental.middle_initial_position);
        controller.set_position(i + 6, Fundamental.lower_initial_position);

def release_dignoal_left_index(controller) :
    for i in Actuator.middle_diagonal_left_index :
        controller.set_position(i, Fundamental.middle_initial_position);
        controller.set_position(i + 6, Fundamental.lower_initial_position);

def walk_forward(controller) :
    controller.set_all_speed(150);
    for i in range(0, 3) :
        __walk_forward_loop(controller);
    
    __walk_forward_right_end(controller);

def __walk_forward_right_end(controller) :
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

def __walk_forward_loop(controller) :
    hold_dignoal_right_index(controller);
    time.sleep(0.1);
    
    # controller.set_all_speed(100);
    for i in Actuator.upper_diagonal_right_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] - 20);
    for i in Actuator.upper_diagonal_left_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] + 20);
    time.sleep(0.2);

    # controller.set_all_speed(50);
    release_dignoal_right_index(controller);
    hold_dignoal_left_index(controller);
    time.sleep(0.1);

    # controller.set_all_speed(100);
    for i in Actuator.upper_diagonal_right_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] + 20);
    for i in Actuator.upper_diagonal_left_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1] - 20);
    time.sleep(0.2);

    # controller.set_all_speed(50);
    release_dignoal_left_index(controller);

    # if not WALK_STOP :
    #     __walk_forward_loop(controller);
    # else :
    #     __walk_forward_right_end(controller);

def walk_backward() :
    print("asdf");

def walk_left() :
    print("asdf");

def walk_right() :
    print("asdf");

def turn_left(controller) :
    hold_dignoal_right_index(controller);
    time.sleep(0.5);

    controller.set_position(Actuator.upper_diagonal_right_index[0], Fundamental.upper_initial_position[Actuator.upper_diagonal_right_index[0] - 1] + 30);
    controller.set_position(Actuator.upper_diagonal_right_index[1], Fundamental.upper_initial_position[Actuator.upper_diagonal_right_index[1] - 1] - 30);
    controller.set_position(Actuator.upper_diagonal_right_index[2], Fundamental.upper_initial_position[Actuator.upper_diagonal_right_index[2] - 1] + 30);
    controller.set_position(Actuator.upper_diagonal_left_index[0], Fundamental.upper_initial_position[Actuator.upper_diagonal_left_index[0] - 1] + 30);
    controller.set_position(Actuator.upper_diagonal_left_index[1], Fundamental.upper_initial_position[Actuator.upper_diagonal_left_index[1] - 1] - 30);
    controller.set_position(Actuator.upper_diagonal_left_index[2], Fundamental.upper_initial_position[Actuator.upper_diagonal_left_index[2] - 1] + 30);
    time.sleep(0.5);

    release_dignoal_right_index(controller);
    hold_dignoal_left_index(controller);
    time.sleep(0.5);

    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    time.sleep(0.5);

    release_dignoal_left_index(controller);

def turn_right(controller) :
    hold_dignoal_left_index(controller);
    time.sleep(0.5);

    controller.set_position(Actuator.upper_diagonal_left_index[0], Fundamental.upper_initial_position[Actuator.upper_diagonal_left_index[0] - 1] + 30);
    controller.set_position(Actuator.upper_diagonal_left_index[1], Fundamental.upper_initial_position[Actuator.upper_diagonal_left_index[1] - 1] - 30);
    controller.set_position(Actuator.upper_diagonal_left_index[2], Fundamental.upper_initial_position[Actuator.upper_diagonal_left_index[2] - 1] + 30);
    controller.set_position(Actuator.upper_diagonal_right_index[0], Fundamental.upper_initial_position[Actuator.upper_diagonal_right_index[0] - 1] + 30);
    controller.set_position(Actuator.upper_diagonal_right_index[1], Fundamental.upper_initial_position[Actuator.upper_diagonal_right_index[1] - 1] - 30);
    controller.set_position(Actuator.upper_diagonal_right_index[2], Fundamental.upper_initial_position[Actuator.upper_diagonal_right_index[2] - 1] + 30);
    time.sleep(0.5);

    release_dignoal_left_index(controller);
    hold_dignoal_right_index(controller);
    time.sleep(0.5);

    for i in Actuator.upper_index :
        controller.set_position(i, Fundamental.upper_initial_position[i - 1]);
    time.sleep(0.5);

    release_dignoal_right_index(controller);


def set_low_walk_mode(controller) :
    low_initial_position(controller);

def low_walk_forward(controller) :
    for i in Actuator.middle_diagonal_left_index :
        controller.set_position(i, 165);

    time.sleep(0.3);

    controller.set_position(1, 165);
    controller.set_position(4, 210);
    controller.set_position(18, 210);

    time.sleep(0.3);
