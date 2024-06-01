from datetime import datetime
import threading
import time
import cv2
from app.Drone import Drone
from app.Camera import Camera
from app.FileManager import FileManager
from app.ArucoDetector import ArucoDetector

class ArucoCentralizer:
    def __init__(self):
        self.processed_image = None
        self.move_x = 0
        self.move_y = 0
        self.tolerance = 40
        self.center_x = 0
        self.center_y = 0
        self.frame = None

    def detect_and_move(self, drone: Drone, camera: Camera, aruco_detector: ArucoDetector):
        c=1
        count = 1
        
        while True:
            if c % 400 ==0:
                print(c)
                camera.read_capture()
                self.frame = camera.frame
                
                if self.frame is None:
                    continue

                # Detectar ArUcos
                self.processed_image, ids, corners = aruco_detector.detect_arucos(self.frame)
                # Mostrar a imagem processada
                # Desenhar linhas guia e o centro do ArUco
                height, width, _ = self.processed_image.shape
                cv2.line(self.processed_image, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)
                cv2.line(self.processed_image, (0, height // 2), (width, height // 2), (0, 255, 0), 2)
                cv2.circle(self.processed_image, (self.center_x, self.center_y), 5, (0, 0, 255), -1)

                cv2.imshow('ArUco Detector', self.processed_image)
                cv2.waitKey(1)
                
                if ids is None or corners is None:
                    print("Nenhum marcador ArUco detectado.")
                    continue

                # Calcular o centro do ArUco detectado
                corner = corners[0][0]
                self.center_x = int((corner[0][0] + corner[2][0]) / 2)
                self.center_y = int((corner[0][1] + corner[2][1]) / 2)

                # Calcular o movimento necessário
                self.move_x = self.center_x - width // 2
                self.move_y = self.center_y - height // 2

                if not 0 in ids and abs(self.move_x) < self.tolerance and abs(self.move_y) < self.tolerance:
    
                    print("ArUco centralizado.")
                    cv2.imwrite(f'centralized/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.jpg', self.processed_image)
                    self.frame_count = 0
                    return True

                # Movimentar o drone baseado na posição do ArUco
                count = count + 1
                if count > 10:
                    move_distance = 2  # Distância de movimento em metros
                    if self.move_x > self.tolerance:
                        drone.move_east(move_distance)
                    elif self.move_x < -self.tolerance:
                        drone.move_west(move_distance)

                    if self.move_y > self.tolerance:
                        drone.move_north(move_distance)
                    elif self.move_y < -self.tolerance:
                        drone.move_south(move_distance)

                camera.clean_buffer()
        
            c+=1    
        return False

    def start(self, drone: Drone, camera: Camera, aruco_detector: ArucoDetector):
        
        self.detect_and_move(drone, camera, aruco_detector)


    def calculate_center_offset(self, corners, frame_shape):
        frame_center = (frame_shape[1] // 2, frame_shape[0] // 2)
        if corners:
            marker_center = corners[0][0].mean(axis=0)
            offset = (marker_center[0] - frame_center[0], marker_center[1] - frame_center[1])
            return offset
        return (0, 0)
    
 