import threading;
import queue;
import time;
import cv2;

from info import *;
from getkey import getkey;
from devices.controller import Controller;
from devices.actuator import Actuator;

def onKeyInput(keyQueue) :
    print("[SYSTEM] Ready for key input\n")
    while (True):
        key = getkey()
        keyQueue.put(key)

def printHelp() :
    print("[SYSTEM] COMMANDS :");
    print("\th : print commands, show this list");
    print("\ti : print information & status");
    print("\tf : change stance");
    print("\t  ex) drive stance -> walk stance")
    print("")

def printInfo() :
    print("[SYSTEM] INFORMATION :");
    print(f"\tName : {NAME}");
    print(f"\tVersion : {VERSION}");
    print("")

if __name__ == "__main__" :
    controller = Controller();

    print("[SYSYEM] SCONE Activated\n");

    for i in Actuator.index :
        controller.set_position(i, Actuator.position.center);

    # keyQueue = queue.Queue()
    # keyThread = threading.Thread(target=onKeyInput, args=(keyQueue,))
    # keyThread.daemon = True 
    # keyThread.start();
    
    # while True :
    #     if keyQueue.qsize() > 0 :
    #         key = keyQueue.get();
            
    #         if key == 'h' :
    #             printHelp();
    #         elif key == 'i' :
    #             printInfo();
    #         elif key == 'q':
    #             print("Exiting serial terminal.");
    #             break;

    #     time.sleep(0.01) 
    # print("End.");
