import cv2
from app.ArucoCentralizer import ArucoCentralizer
from app.Drone import Drone
from app.Camera import Camera
from app.ArucoDetector import ArucoDetector

import time

def main():
    drone = Drone()
    camera = Camera()
    aruco_detector = ArucoDetector()
    centralizer = ArucoCentralizer()
    
    frame_skip = 5  # Process every 5th frame
    frame_count = 0
    
    if not drone.connected():
        print("Falha ao conectar com o drone.")
        return

    camera.initialize_video_capture('computer')

    drone.solicit_telemetry()
    drone.change_to_guided_mode()
    drone.arm_drone()
    drone.ascend(4)
  
    try:
        i = 1
        while True:
            
            i = 1 + i
            camera.read_capture()
            
            if not camera.ret:
                continue

            cv2.imshow('Video', camera.frame)
            
            frame_count += 1    
            if frame_count % frame_skip == 0:
                image, _, __ = aruco_detector.detect_arucos(camera.frame)
                
                centralizer.detect_and_move(drone, camera, aruco_detector)
                
                cv2.imshow('Video', image)

            aruco_detector.go_to_aruco_direction(drone)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except KeyboardInterrupt:
        drone.land()
        drone.disarm()
        print('Desceu')
        print("Simulação interrompida pelo usuário.")
        
    finally:
        if camera.cap:
            camera.cap.release()
        
        drone.land()
        drone.disarm()
        cv2.destroyAllWindows()
        print('Desceu')

    
        
if __name__ == "__main__":
    main()
