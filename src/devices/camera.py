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