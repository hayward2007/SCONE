import os;
import sys;
import time;

from dynamixel_sdk import *;

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


import imutils  # 이미지 처리 유틸리티 라이브러리 임포트
from imutils.video import VideoStream, FPS  # 비디오 스트리밍 및 FPS 측정을 위한 라이브러리 임포트
from multiprocessing import Process, Queue  # 멀티프로세싱을 위한 라이브러리 임포트
import numpy as np  # 수치 계산을 위한 라이브러리 임포트
import time  # 시간 관련 기능을 위한 라이브러리 임포트
import cv2  # OpenCV 라이브러리 임포트
# 카메라 클래스
class Camera :
    __CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"];
    
    # 클래스별 색상 랜덤 지정
    __COLORS = np.random.uniform(0, 255, size=(len(__CLASSES), 3));
    
    def __init__(self) :
        print("[SYSTEM] CAMERA INIT...");
        self.__configure_model();
        self.start();
    
    def __configure_model(self) :
        self.net = cv2.dnn.readNetFromCaffe("./MobileNetSSD_deploy.prototxt.txt", "./MobileNetSSD_deploy.caffemodel");
        self.input_queue = Queue(maxsize=1);
        self.output_queue = Queue(maxsize=1);
        self.detections = None;
    
    def __classify_frame(self, net, input_queue, output_queue) :
        while True :
            if not input_queue.empty() :
                frame = input_queue.get()
                frame = cv2.resize(frame, (300, 300))
                blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
                net.setInput(blob)
                detections = net.forward()
                output_queue.put(detections)

    # 객체 감지 메서드
    def detect_object(self):
        frame = self.video.read()
        frame = imutils.resize(frame, width=400)
        (height, width) = frame.shape[:2]
        if self.input_queue.empty():
            self.input_queue.put(frame)
        if not self.output_queue.empty():
            self.detections = self.output_queue.get()
        if self.detections is not None:
            for i in np.arange(0, self.detections.shape[2]):
                confidence = self.detections[0, 0, i, 2]
                idx = int(self.detections[0, 0, i, 1])
                dims = np.array([width, height, width, height])
                box = self.detections[0, 0, i, 3:7] * dims
                (startX, startY, endX, endY) = box.astype("int")
                label = "{}: {:.2f}%".format(self.__CLASSES[idx], confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY), self.__COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.__COLORS[idx], 2)
        cv2.imshow("Frame", frame)
        self.fps.update()
    
    def start(self):
        print("[CAMERA] starting process...")
        self.process = Process(target=self.__classify_frame, args=(self.net, self.input_queue, self.output_queue))
        self.process.daemon = True
        self.process.start()
        print("[CAMERA] starting video stream...")
        self.video = VideoStream(src=0).start()
        time.sleep(2.0)
        self.fps = FPS().start()
    
    def stop(self):
        self.fps.stop()
        print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
        cv2.destroyAllWindows()
        self.video.stop()