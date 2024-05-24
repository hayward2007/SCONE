import threading;
import queue;
import time;
import cv2;

from info import *;
from getkey import getkey;

from devices.actuator import Actuator;
from devices.controller import Controller;

controller = Controller();

def on_key_input(keyQueue) :
    print("[SYSTEM] Ready for key input\n")
    while (True):
        key = getkey()
        keyQueue.put(key)

def printHelp() :
    print("----------- BASICS -----------");
    print("h : print commands, show this list");
    print("i : print information & status");
    print("f : change stance");
    print("o : disable / enable torque\n");
    print("---------- MOVEMENTS ----------");
    print("w : walk forward");
    print("d : walk backward");
    print("a : turn left");
    print("s : turn right\n");
    # print("\t\tex) drive stance -> walk stance")

def disable_torque() :
    for id in Actuator.index :
        controller.set_torque(id, 0);

if __name__ == "__main__" :
    controller = Controller();
    
    print("[SYSYEM] SCONE Activated\n");

    keyQueue = queue.Queue()
    keyThread = threading.Thread(target=on_key_input, args=(keyQueue,))
    keyThread.daemon = True 
    keyThread.start();
    
    while True :
        if keyQueue.qsize() > 0 :
            key = keyQueue.get();
            
            if key == 'h' :
                print_help();
            elif key == 'i' :
                print_info();
            elif key == 'o' :
                disable_torque();
            elif key == 'q':
                print("Exiting serial terminal.");
                break;

        time.sleep(0.01) 
    print("End.");
