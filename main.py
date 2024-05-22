import os;
import sys;
from dynamixel_sdk import *;
import time;

class scone :
    class dynamixel :
        class __AX_18A :
            CENTER = 512;
            def __init__(self) :
                print("AX-18A");

        class __MX_28AT :
            CENTER = 2048;
            def __init__(self) :
                print("MX-28AT");

        index = {
            1 : __AX_18A,
            2 : __AX_18A,
            3 : __AX_18A,
            4 : __AX_18A,
            5 : __AX_18A,
            6 : __AX_18A,
            7 : __MX_28AT,
            8 : __MX_28AT,
            9 : __MX_28AT,
            10 : __MX_28AT,
            11 : __MX_28AT,
            12 : __MX_28AT,
            13 : __AX_18A,
            14 : __AX_18A,
            15 : __AX_18A,
            16 : __AX_18A,
            17 : __AX_18A,
            18 : __AX_18A,
        };

        upper_right_index = [1, 4, 5];
        upper_left_index = [2, 3, 6];
        middle_right_index = [i + 6 for i in upper_right_index];
        middle_left_index = [i + 6 for i in upper_left_index];

        speed = 100;

        __BAUDRATE = 1000000;
        __DEVICENAME = '/dev/ttyUSB0';
        __PROTOCOL_VERSION = 1.0;

        def __init__(self) :
            # Initialize port
            self.port_handler = PortHandler(self.__DEVICENAME);
            self.packet_handler = PacketHandler(self.__PROTOCOL_VERSION);
            # Open the port
            if self.port_handler.openPort() :
                print("Succeeded to open the port");
            else :
                print("Failed to open the port");
                exit(1);
            # Set the baudrate
            if self.port_handler.setBaudRate(self.__BAUDRATE) :
                print("Succeeded to set the baudrate");
            else :
                print("Failed to set the baudrate");
                exit(1);
            # Set motor speed to 200
            for i in self.index :
                self.set_speed(i, self.speed);
        
        def __del__(self) :
            self.port_handler.closePort();

        def set_speed(self, id, speed) :
            self.packet_handler.write2ByteTxRx(self.port_handler, id, 32, speed);
        
        def set_torque(self, id, status) :
            self.packet_handler.write1ByteTxRx(self.port_handler, id, 24, status);

        def set_position(self, id, position) :
            self.packet_handler.write2ByteTxRx(self.port_handler, id, 30, position);

        def get_position(self, id) :
            result, data, error = self.packet_handler.read2ByteTxRx(self.port_handler, id, 36);
            print(f"Motor {id} Current Position: {result}");
            return result;
        
    def __init__(self) :
        print("Port closed");
        self.robot = self.dynamixel();
        self.initialize_position();

    def turn_off(self) :
        for i in self.robot.index :
            self.robot.set_torque(i, 0);
    
    def goal_position(self, id, gap) :
        return self.robot.index[id].CENTER - (gap if id % 2 else - gap);

    def hold_right(self) :
        for i in self.robot.middle_right_index :
            self.robot.set_position(i, self.goal_position(i, 600));
    
    def release_right(self) :
        for i in self.robot.middle_right_index :
            self.robot.set_position(i, self.goal_position(i, 200));
    
    def hold_left(self) :
        for i in self.robot.middle_left_index :
            self.robot.set_position(i, self.goal_position(i, 600));
    
    def release_left(self) :
        for i in self.robot.middle_left_index :
            self.robot.set_position(i, self.goal_position(i, 200));

    def center_position(self) :
        for i in self.robot.index :
            self.robot.set_position(i, self.robot.index[i].CENTER);

    def initialize_position(self) :
        gap = 200;
        for i in self.robot.index :
            goal_position = self.robot.index[i].CENTER;
            if i > 6 and i % 2 :
                goal_position -= gap;
            elif i > 6 :
                goal_position += gap;
            self.robot.set_position(i, goal_position);

    def stand_position(self) :
        gap = 1364;
        for i in self.robot.index :
            goal_position = self.robot.index[i].CENTER;
            if i > 6 and i % 2 :
                goal_position = 4096 - gap;
            elif i > 6 :
                goal_position = gap;
            self.robot.set_position(i, goal_position);
    
    def walk_forward(self) :
        self.hold_right();

        time.sleep(0.5);
        
        for i in self.robot.upper_left_index :
            gap = 100;
            temp = self.robot.index[i].CENTER + (gap if i % 2 else - gap);
            self.robot.set_position(i, temp);
        
        for i in self.robot.upper_right_index :
            self.robot.set_position(i, self.robot.index[i].CENTER);

        time.sleep(0.5);
    
        self.release_right();
        self.hold_left();
    
        time.sleep(0.5);

        for i in self.robot.upper_right_index :
            gap = 100;
            temp = self.robot.index[i].CENTER + (gap if i % 2 else - gap);
            self.robot.set_position(i, temp);
    
        for i in self.robot.upper_left_index :
            self.robot.set_position(i, self.robot.index[i].CENTER);
    
        time.sleep(0.5);

        self.release_left();
        self.walk_forward();
    
    def release_right_high(self) :
        for i in self.robot.middle_right_index :
            self.robot.set_position(i, self.goal_position(i, -684));
    
    def release_left_high(self) :
        for i in self.robot.middle_left_index :
            self.robot.set_position(i, self.goal_position(i, -684));

    def walk_forward_high(self) :
        self.hold_right();
        time.sleep(0.5);

        for i in self.robot.upper_left_index :
            gap = 100;
            temp = self.robot.index[i].CENTER + (gap if i % 2 else - gap);
            self.robot.set_position(i, temp);
        for i in self.robot.upper_right_index :
            self.robot.set_position(i, self.robot.index[i].CENTER);
        time.sleep(0.5);

        self.release_right_high();
        time.sleep(1);
        self.hold_left();
        time.sleep(0.5);
    
        for i in self.robot.upper_right_index :
                gap = 100;
                temp = self.robot.index[i].CENTER + (gap if i % 2 else - gap);
                self.robot.set_position(i, temp);
        for i in self.robot.upper_left_index :
            self.robot.set_position(i, self.robot.index[i].CENTER);
        time.sleep(0.5);
    
        self.release_left_high();
        time.sleep(1);
        self.walk_forward_high();


    def turn_left(self) :
        self.hold_right();
        
        time.sleep(0.5);

        for i in range(1, 7) :
            self.robot.set_position(i, self.robot.index[i].CENTER + 200);
        



def check_path() :
    path = ["device", "sensor", "camera"];
    for i in range(0, len(path)) :
        path[i] = os.path.abspath("./" + path[i]);
    if not path in sys.path :
        for i in path :
            sys.path.append(i);

    import device;


if __name__ == "__main__" :
    print("hello")
    while(True) :
        try :
            print("hasdf");
        except Exception as error :
            print(error);
            print("[SYSTEM] Executing program");
            exit(1);

        
        
    # check_path();
    # scone = scone();
    # while True :
    #     try :
    #         time.sleep(1);
    #         # scone.stand_position();
    #         # time.sleep(1);
    #         # scone.walk_forward_high();
    #         # scone.walk_forward();
    #         exit(1);
    #     except KeyboardInterrupt :
    #         scone.turn_off();
    #         del scone;
    #         print("Program terminated");
    #         exit(1);

