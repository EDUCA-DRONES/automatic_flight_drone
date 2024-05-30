from datetime import datetime
import cv2
import time
from app.Drone import Drone
from app.FileManager import FileManager
from app.ArucoDetector import ArucoDetector

class CameraPresenter:
    def init_message(self):
        print("Inicializando captura de vídeo...")

class CameraConnection:
    source = None
    presenter = CameraPresenter()
    
    def connection(self):
        self.presenter.init_message()
        return cv2.VideoCapture(int(self.source) if self.source.isdigit() else self.source)
        
class CameraRTSP(CameraConnection):
    source = 'rtsp://admin:admin@192.168.0.111:554/11'

class CameraComputer(CameraConnection):
    source = '0'

class CameraIMX519(CameraConnection):
    source = 'tcp://192.168.0.101:8554'

class CameraESP32CAM(CameraConnection):
    source = 'http://192.168.0.108:81/stream'

class CameraAnalog(CameraConnection):
    source = '/dev/video2'

class CameraConnectionFactory():
    
    @staticmethod
    def create(type) -> CameraConnection: 
        cameras = {
            'rtsp' : CameraRTSP,
            'computer': CameraComputer,
            'imx': CameraIMX519,
            'analog': CameraAnalog,
            'esp32': CameraESP32CAM

        }
           
        return cameras[type]()

class Camera:
    def __init__(self) -> None:
        
        self.presenter = CameraPresenter()
        self.cap = None
        self.ret = None
        self.frame = None

    def initialize_video_capture(self, type):
        camera = CameraConnectionFactory.create(type)
        
        self.cap = camera.connection()
        
        if not self.cap.isOpened():
            print("Falha ao abrir a captura de vídeo.")
        
    def read_capture(self):
        self.ret, self.frame = self.cap.read()

       
    def capture_images_and_metadata(self, drone: Drone,  fileManager: FileManager, aruco_detector: ArucoDetector, alt, num_images=3, pause_time=3):
       
        for i in range(num_images):
            # time.sleep(pause_time)  # Espera entre capturas

            ret, frame = self.cap.read()
            processed_image, _, __ = aruco_detector.detect_arucos(frame)
            lat, long, alt_ = drone.get_gps_position()
            
            if ret:
                fileManager.create_image(processed_image, alt, i)
                fileManager.create_meta_data(lat, long, alt, drone.current_altitude(), i)
                    
            for _ in range(40):  # Limpa os últimos 40 frames para garantir frescor
                self.cap.grab()
