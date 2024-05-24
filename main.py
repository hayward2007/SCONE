import threading;
import queue;
import time;
import cv2;
from getkey import getkey;

# __CHANGE = 'f';
# __INFO = 'i';
# __HELP = 'h';
__EXIT = 'q';


def onKeyInput(keyQueue) :
    print('Ready for key input:')
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


if __name__ == "__main__" :

    keyQueue = queue.Queue()
    keyThread = threading.Thread(target=onKeyInput, args=(keyQueue,))
    keyThread.daemon = True 
    keyThread.start()

    print("[SYSYEM] SCONE Activated");
    # print("[SYSYEM] Ready for key input");
    printHelp();
    while (True):
        if (keyQueue.qsize() > 0):
            key = keyQueue.get()
            # print("key = {}".format(key))
            if (key == 'h') :
                printHelp();
            elif (key == 'q'):
                print("Exiting serial terminal.")
                break

        time.sleep(0.01) 
    print("End.");