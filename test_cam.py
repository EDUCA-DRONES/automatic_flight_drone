from datetime import datetime

import cv2
from app.ArucoDetector import ArucoDetector
from app.Camera import Camera
from app.FileManager import FileManager

aruco_detector = ArucoDetector()
fileManager = FileManager()

def main():
    camera = Camera()
        
    camera.initialize_video_capture('rtsp')
    
    try:
        while True:
            camera.read_capture()
            
            camera.save_image(f'centralized/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.jpg')
            cv2.imshow('Teste', camera.frame)
            cv2.waitKey(1)
    except:
        if camera.cap:
            camera.cap.release()
        
        cv2.destroyAllWindows()
    
    
main()