
import cv2
from app.ArucoCentralizer import ArucoCentralizer
from app.ArucoDetector import ArucoDetector
from app.Camera import Camera
from app.Drone import Drone


def main():
    drone = Drone()
    camera = Camera()
    aruco_centralizer = ArucoCentralizer(drone, camera)
    
    try: 
        camera.initialize_video_capture('computer')

        if not drone.connected():
            print("Falha na conexão com o drone.")
            return 
        else:
            drone.change_to_guided_mode()
            drone.arm_drone()
            drone.ascend(6)  # Subir para 6 metros
            # drone.ascend(20)  # Subir para 6 metros
            aruco_centralizer.execute()
        
    except Exception as e:
        print(e)
    finally:
        drone.land()
        drone.disarm()
        camera.cap.release()
        drone.conn.close()
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
   main()