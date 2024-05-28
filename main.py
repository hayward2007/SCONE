import threading, time;
from getch import getch;

from info import *;
from motions.walk import *;
from motions.drive import *;
from motions.stair import *;
from devices.actuator import *;
from devices.controller import *;
from motions.fundamental import *;


controller = Controller();

def cli() :
    def cli_input() :
        print("");
        user_input = input(">> ");
        print("");
        return user_input;

    def is_return(input) :
        input = input.lower();
        return input == "quit" or input == "exit" or input == "return";

    def no_command() :
        print("[SYSTEM] Invalid command, type 'help' for command list");
        print("");
        return;

    def print_help() :
        print("[SYSTEM] Command list");
        print("  help : show command list");
        print("  info : show information & status");
        print("  set  : set system preferences");
        print("  get  : get system status");
        print("  remote : remote control scone");
        print("");
        return;
    
    def print_info() :
        print("[SYSTEM] Information");
        print(f"  name : {NAME}");
        print(f"  version : {VERSION}");
        print("");
        return;

    def cli_set() :
        def scone() :
            print("Asdf");
        
        def actuator() :
            def speed() :
                print("[SYSTEM] Set actuator speed?");
                print(" 1. Slow");
                print(" 2. Normal");
                print(" 3. Fast");
                print(" 4. Custom");
                print(" 5. Exit");
                print("");

            print("[SYSTEM] Set actuator?");
            print(" 1. Torque");
            print(" 2. Position");
            print(" 3. Speed");
            print("");
        
        print("[SYSTEM] Set?");
        print(" 1. SCONE");
        print(" 2. Actuator");
        print(" 3. Return");
        print("");

        user_input = cli_input();

        if user_input == "1" or user_input.lower == "scone" :
            scone();
        elif user_input == "2" or user_input.lower == "actuator" :
            actuator();
        elif is_return(user_input) :
            return;
        else :
            no_command();
            cli_set();
    
    def cli_get() :
        print("[SYSTEM] Get?");
        print(" 1. Torque");
        print(" 2. Position");
        print(" 3. Speed");
        print("");
    
    def cli_remote() :
        remote_input = "";
        remote = True;
    
        while remote :
            remote_input = getch();
            print(remote_input);
            if remote_input == "q" :
                remote = False;
                return;
            elif remote_input == "w" :
                print("w");
            elif remote_input == "a" :
                print("a");
            elif remote_input == "s" :
                print("s");
            elif remote_input == "d" :
                print("d");
            elif remote_input == " " :
                print(" ");
            else :
                print("Invalid command");
                print("");
            
            time.sleep(0.01);

    user_input = cli_input();

    if user_input == "help" :
        print_help();
    
    elif user_input == "info" :
        print_info();
    
    elif user_input == "set" :
        cli_set();
    
    elif user_input == "get" :
        cli_get();
    
    elif user_input == "remote" :
        cli_remote();
        
    elif is_return(user_input) :
        return;
    
    else :
        no_command();
    
    cli();

if __name__ == "__main__" :
    print("[SYSYEM] SCONE Activated\n");
    cli();
    print("[SYSYEM] Bye.\n");


