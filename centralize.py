
import cv2
from app.ArucoCentralizer import ArucoCentralizer
from app.ArucoDetector import ArucoDetector
from app.Camera import Camera
from app.Drone import Drone
from app.CameraCalibrator import CameraCalibrator

def main():
    drone = Drone()
    camera = Camera()
    aruco_detector = ArucoDetector()
    camera_calibrator = CameraCalibrator()
    camera_matrix, dist_coeffs = camera_calibrator.calibrate()
    
    try: 
        camera.initialize_video_capture('rtsp')

        if not drone.connected():
            print("Falha na conexão com o drone.")
            return 
        else:
            drone.change_to_guided_mode()
            drone.arm_drone()
            drone.ascend(6)  # Subir para 6 metros
            # drone.ascend(20)  # Subir para 6 metros

        count=0
        while True:
            camera.read_capture()
            if not camera.ret:
                break
            
            positions =  aruco_detector.get_aruco_positions(camera.frame, camera_matrix, dist_coeffs)
            
            if positions:
                count += 1
                if count > 100:
                    drone.adjust_position(positions.get('x'), positions.get('y'))
                    break
            else:
                count = 0

            height, width, _ = camera.frame.shape
            cv2.line(camera.frame, (width // 2, 0), (width // 2, height), (0, 0, 255), 2)
            cv2.line(camera.frame, (0, height // 2), (width, height // 2), (0, 0, 255), 2)
            cv2.circle(camera.frame, (0, 0), 5, (0, 0, 255), -1)

            cv2.imshow('Frame', camera.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


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
 