import threading;
import queue;
import time;
import cv2;
from getkey import getkey;


def onKeyInput(keyQueue):
    print('Ready for key input:')
    while (True):
        key = getkey()
        keyQueue.put(key)

if __name__ == "__main__" :
    EXIT_COMMAND = 'q';

    keyQueue = queue.Queue()
    keyThread = threading.Thread(target=onKeyInput, args=(keyQueue,))
    keyThread.daemon = True 
    keyThread.start()

    while (True):
        if (keyQueue.qsize() > 0):
            key = keyQueue.get()
            print("key = {}".format(key))

            if (key == EXIT_COMMAND):
                print("Exiting serial terminal.")
                break

        time.sleep(0.01) 
    print("End.");