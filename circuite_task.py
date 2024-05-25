from app.Drone import Drone
from app.Camera import Camera
from app.ArucoDetector import ArucoDetector
import cv2

CAMERA_INDEX = '0'

def main():
    drone = Drone()
    camera = Camera()
    aruco_detector = ArucoDetector()
    
    if not drone.connected():
        print("Falha ao conectar com o drone.")
        return

    drone.solicit_telemetry()
    camera.initialize_video_capture('computer')
    
    try:
        i = 1
        while True:
            i = 1 + i
            camera.read_capture()
            
            if not camera.ret:
                continue
            # cv2.imwrite('test.jpg', camera.frame)
            cv2.imshow('Video', camera.frame)
            
            # if i > 10:
            #     break
            
            aruco_detector.detect_arucos(camera.frame)
            
            if aruco_detector.ids is not None:
                if 33 in aruco_detector.ids:
                    print("ArUco 33 detectado, alterando para modo GUIDED e preparando para decolagem...")
                    drone.change_to_guided_mode()
                    drone.arm_drone()
                    drone.takeoff(10)
                    
                    # drone.land()
                    # drone.disarm()
                    break

                elif 8 in aruco_detector.ids:
                    print("ArUco 8 detectado, iniciando procedimentos de pouso...")
                    drone.land()
                    drone.disarm()
                    
                    break
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except KeyboardInterrupt:
        if drone.current_altitude():
            drone.land()
            drone.disarm()
            print('Desceu')
        print("Simulação interrompida pelo usuário.")
        
    finally:
        if camera.cap:
            camera.cap.release()
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    main()
