import threading;
import queue;
import time;
import cv2;

from info import *;
from getkey import getkey;

from motions.walk import *;
from motions.drive import *;
from motions.fundamental import *;
from devices.actuator import Actuator;
from devices.controller import Controller;

controller = Controller();
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

def remote() :
    remote = True;
    keyQueue = queue.Queue()
    keyThread = threading.Thread(target=on_key_input, args=(keyQueue,))
    keyThread.daemon = True; 
    keyThread.start();
    print("[SYSTEM] Remote control activated\n");
    while True :
        if keyQueue.qsize() > 0 :
            key = keyQueue.get();
            if key == 'r' :
                set_drive_mode(controller);
            elif key == 'w' :
                drive_forward(controller);
            elif key == ' ' :
                drive_stop(controller);
            elif key == 'o' :
                disable_torque();
            elif key == 'q':
                print("[SYSTEM] Exiting serial terminal");
                break;
        time.sleep(0.01) 
    remote = False;

def command_line_interface() :
    user_input = input("[SYSTEM] Enter command : ");
    if user_input == "quit" or user_input == "exit" :
        return;
    elif user_input == "help" :
        print_help();
    elif user_input == "info" :
        print_info();
    elif user_input == "remote" :
        remote();
    elif user_input == "torque off" :
        disable_torque(controller);
    else :
        print("[SYSTEM] Invalid command, type 'help' for command list\n");
    command_line_interface();

if __name__ == "__main__" :
    print("[SYSYEM] SCONE Activated\n");
    command_line_interface();
    print("[SYSYEM] Bye.\n");
