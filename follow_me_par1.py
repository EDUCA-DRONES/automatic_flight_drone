import cv2
from app.Drone import Drone
from app.Camera import Camera
from app.ArucoDetector import ArucoDetector

import time

def main():
 #   drone = Drone()
    camera = Camera()
    aruco_detector = ArucoDetector()

    '''
        frame_skip = 5  # Process every 5th frame
        frame_count = 0
        
        if not drone.connected():
            print("Falha ao conectar com o drone.")
            return
    '''
    
    #camera.initialize_video_capture('computer')

    camera_types = ['imx' ]
    cameras = {type: Camera() for type in camera_types}

    for camera_type, camera in cameras.items():
        print(f"Iniciando captura de imagens da câmera tipo: {camera_type}")
        camera.initialize_video_capture(camera_type)
        

    '''
    drone.solicit_telemetry()
    drone.change_to_guided_mode()
    drone.arm_drone()
    drone.takeoff(10)
    '''
  
    try:
        i = 1
        while True:
            i = 1 + i
            camera.read_capture()
            
            if not camera.ret:
                continue

            cv2.imshow('Video', camera.frame)
            
            ''''
            frame_count += 1    
            if frame_count % frame_skip == 0:
                cv2.imshow('Video', camera.frame)
                aruco_detector.detect_arucos(camera.frame)

            aruco_detector.go_to_aruco_direction(drone)
            '''
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except KeyboardInterrupt:
        '''
        if drone.current_altitude():
            drone.land()
            drone.disarm()
            print('Desceu')
        print("Simulação interrompida pelo usuário.")
        '''
        
    finally:
        if camera.cap:
            camera.cap.release()
        cv2.destroyAllWindows()

     
   
        
        
if __name__ == "__main__":
    main()
