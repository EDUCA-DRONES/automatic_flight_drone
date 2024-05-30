from app.ArucoDetector import ArucoDetector
from app.Camera import Camera
from app.FileManager import FileManager

aruco_detector = ArucoDetector()
fileManager = FileManager()

def main():
    
    while True:
        camera = Camera()
        
        camera.initialize_video_capture('rtsp')
        print(f"Iniciando captura de imagens da câmera tipo: {'rtsp'}")
    
        

main()