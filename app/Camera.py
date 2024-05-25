import cv2

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
    source = 'rtsp://admin:admin@192.168.0.100:554/11'

class CameraComputer(CameraConnection):
    source = '0'

class CameraConnectionFactory():
    
    @staticmethod
    def create(type) -> CameraConnection: 
        cameras = {
            'rtsp' : CameraRTSP,
            'computer': CameraComputer
        }
           
        return cameras[type]

class Camera:
    def __init__(self) -> None:
        
        self.presenter = CameraPresenter()
        self.cap = None
        self.ret = None
        self.frame = None

    def initialize_video_capture(self, type):
        camera = CameraConnectionFactory.create(type)()
        
        self.cap = camera.connection()
        
        if not self.cap.isOpened():
            print("Falha ao abrir a captura de vídeo.")
        
    def read_capture(self):
        self.ret, self.frame = self.cap.read()
       
