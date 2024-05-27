import threading;
import queue;
import time;
import cv2;

from info import *;
from getkey import getkey;

from motions.walk import *;
from motions.drive import *;
from motions.stair import *;
from devices.actuator import *;
from devices.controller import *;
from motions.fundamental import *;

# controller = Controller();
remote = False;

def on_key_input(keyQueue) :
    print("[SYSTEM] Ready for key input\n")
    while (remote):
        key = getkey()
        keyQueue.put(key)

def print_help() :
    print("----------- BASICS -----------");
    print("h : print commands, show this list");
    print("i : print information & status");
    print("f : change stance");
    print("  ex) drive stance -> walk stance")
    print("o : disable / enable torque\n");
    print("---------- MOVEMENTS ----------");
    print("w : walk forward");
    print("d : walk backward");
    print("a : turn left");
    print("s : turn right\n");

def print_info() :
    print(f"Name : {NAME}");
    print(f"Version : {VERSION}");

def get_position(controller) :
    for i in Actuator.index :
        controller.get_position(i);

# def remote() :
#     remote = True;
#     keyQueue = queue.Queue()
#     keyThread = threading.Thread(target=on_key_input, args=(keyQueue,))
#     keyThread.daemon = True; 
#     keyThread.start();
#     print("[SYSTEM] Remote control activated\n");
#     while remote :
#         if keyQueue.qsize() > 0 :
#             key = keyQueue.get();
#             if key == 'r' :
#                 set_drive_mode(controller);
#             elif key == 'w' :
#                 drive_forward(controller);
#             elif key == ' ' :
#                 drive_stop(controller);
#             elif key == 'o' :
#                 disable_torque();
#             elif key == 'q':
#                 print("[SYSTEM] Exiting serial terminal");
#                 break;
#         time.sleep(0.01) 
#     remote = False;

def command_line_interface() :
    user_input = input("[SYSTEM] Enter command : ");
    if user_input == "quit" or user_input == "exit" :
        return;
    elif user_input == "help" :
        print_help();
    
    elif user_input == "info" :
        print_info();
    
    # elif user_input == "remote" :
    #     remote();

    elif user_input == "walk mode" :
        set_walk_mode(controller);

    elif user_input == "walk forward" :
        walk_forward(controller);
    
    elif user_input == "drive mode" :
        set_drive_mode(controller);
    
    elif user_input == "drive forward" :
        drive_forward(controller);
    
    elif user_input == "drive stop" :
        drive_stop(controller);
    
    elif user_input == "stair mode" :
        set_stair_mode(controller);
    
    elif user_input == "climb stair" :
        climb_stair(controller);
    
    elif user_input == "turn right" :
        turn_right(controller);
    
    elif user_input == "turn left" :
        turn_left(controller);
    
    elif user_input == "position" :
        get_position(controller);
    
    elif user_input == "torque off" :
        disable_torque(controller);
    
    elif user_input == "set" :
        print("[SYSTEM] Set?");
        print(" 1. Torque");
        print(" 2. Position");
        print(" 3. Speed\n");
        print("");
        additional_user_input = input(" >");


    elif user_input == "get" :
        print("[SYSTEM] Get?");
        print(" 1. Torque");
        print(" 2. Position");
        print(" 3. Speed");
    
    else :
        print("[SYSTEM] Invalid command, type 'help' for command list\n");
    
    command_line_interface();

if __name__ == "__main__" :
    print("[SYSYEM] SCONE Activated\n");
    command_line_interface();
    print("[SYSYEM] Bye.\n");
