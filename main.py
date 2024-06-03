import time;
from motions.walk import *;
from motions.drive import *;
from motions.stair import *;
from devices.actuator import *;
from devices.controller import *;
from motions.fundamental import *;


# status = Status.INITIALIZING;
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
        print(f"  name : SCONE v2");
        print(f"  version : 2.0.0");
        print("");
        return;

    def cli_set() :
        def set_posture() :
            print("Asdf");
        
        def set_actuator() :
            def set_torque() :
                print("[SYSTEM] Set actuator torque?");
                print(" 1. On");
                print(" 2. Off");
                print(" 3. Exit");
                print("");
            
                user_input = cli_input();

                if user_input == "1" or user_input.lower() == "on" :
                    print("[SYSTEM] Enter actuator id");
                    print("");
                    controller.set_torque(int(cli_input()), True);
                elif user_input == "2" or user_input.lower() == "off" :
                    print("[SYSTEM] Enter actuator id");
                    print("");
                    controller.set_torque(int(cli_input()), False);
                elif is_return(user_input) :
                    return;
                else :
                    no_command();
                    cli_set();
            
            def set_position() :
                print("[SYSTEM] Set actuator position?");
                print(" 1. Standby");
                print(" 2. Walk");
                print(" 3. Drive");
                print(" 4. Stair");
                print(" 5. Exit");
                print("");
            
                # user_input = cli_input();

            
            def set_speed() :
                print("[SYSTEM] Set actuator speed?");
                print(" 1. All");
                print(" 2. Normal");
                print(" 5. Exit");
                print("");

            print("[SYSTEM] Set actuator?");
            print(" 1. Torque");
            print(" 2. Position");
            print(" 3. Speed");
            print("");
        
            user_input = cli_input();

            if user_input == "1" or user_input.lower() == "torque" :
                set_torque();
            elif user_input == "2" or user_input.lower() == "position" :
                set_position();
            elif user_input == "3" or user_input.lower() == "speed" :
                set_speed();
            elif is_return(user_input) :
                return;
            else :
                no_command();
                cli_set();
        
        print("[SYSTEM] Set?");
        print(" 1. SCONE");
        print(" 2. Actuator");
        print(" 3. Exit");
        print("");

        user_input = cli_input();

        if user_input == "1" or user_input.lower == "scone" :
            set_posture();
        elif user_input == "2" or user_input.lower == "actuator" :
            set_actuator();
        elif is_return(user_input) :
            return;
        else :
            no_command();
            cli_set();
    
    def cli_get() :
        def get_torque() :
            def get_all_torque() :
                for i in Actuator.index :
                    controller.get_torque(i);
            
            def get_one_torque() :
                print("[SYSTEM] Enter actuator id");
                print("");
                controller.get_torque(int(cli_input()));
            
            print("[SYSTEM] Get torque?");
            print(" 1. All");
            print(" 2. One");
        
            user_input = cli_input();

            if user_input == "1" or user_input.lower == "all" :
                get_all_torque();
            elif user_input == "2" or user_input.lower == "one" :
                get_one_torque();

        def get_position() :
            def get_all_position() :    
                for i in Actuator.index :
                    controller.get_position(i);
            
            def get_one_position() :
                print("[SYSTEM] Enter actuator id");
                print("");
                controller.get_position(int(cli_input()));
        
            print("[SYSTEM] Get position?");
            print(" 1. All");
            print(" 2. One");
        
            user_input = cli_input();
        
            if user_input == "1" or user_input.lower == "all" :
                get_all_position();
            elif user_input == "2" or user_input.lower == "one" :
                get_one_position();

        def get_speed() :
            def get_all_speed() :
                for i in Actuator.index :
                    controller.get_speed(i);
            
            def get_one_speed() :
                print("[SYSTEM] Enter actuator id");
                print("");
                controller.get_speed(int(cli_input()));
            
            print("[SYSTEM] Get speed?");
            print(" 1. All");
            print(" 2. One");
            print(" 3. Exit");
            print("");

            user_input = cli_input();

            if user_input == "1" or user_input.lower == "all" :
                get_all_speed();
            elif user_input == "2" or user_input.lower == "one" :
                get_one_speed();


        print("[SYSTEM] Get?");
        print(" 1. Torque");
        print(" 2. Position");
        print(" 3. Speed");
        print(" 4. Exit");
        print("");

        user_input = cli_input();
    
        if user_input == "1" or user_input.lower == "torque" :  
            get_torque();
        elif user_input == "2" or user_input.lower == "position" :
            get_position();
        elif user_input == "3" or user_input.lower == "speed" :
            get_speed();
        elif is_return(user_input) :
            return;
        else :
            no_command();
            cli_get();
    
    def cli_remote() :
        # global status;
        remote_input = "";
        remote = True;
    
        while remote :
            remote_input = getch();
            print(remote_input);
            if remote_input == "q" :
                remote = False;
                return;
            elif remote_input == "w" :
                walk_forward(controller);
                # status = Status.WALKING;
                print("asdf");
            elif remote_input == "a" :
                # status = Status.STANDBY;
                turn_left(controller);
            elif remote_input == "s" :
                print("s");
            elif remote_input == "d" :
                turn_right(controller);
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
    
    elif user_input == "right" :
        turn_right(controller);

    elif user_input == "left" :
        turn_left(controller);
    
    elif user_input == "drive mode" :
        set_drive_mode(controller);
    
    elif user_input == "drive" :
        drive_forward(controller);

    elif user_input == "walk mode" : 
        set_walk_mode(controller);

    elif user_input == "walk" : 
        walk_forward(controller);
    
    elif user_input == "stair mode" :
        set_stair_mode(controller);
    
    elif user_input == "climb" :
        climb_stair(controller);
        
    elif is_return(user_input) :
        end_position(controller);
        return;
    
    else :
        no_command();
    
    cli();

if __name__ == "__main__" :
    # status = Status.STANDBY;
    print("[SYSYEM] SCONE Activated\n");
    cli();
    print("[SYSYEM] Bye.\n");


