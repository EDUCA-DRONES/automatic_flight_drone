import cv2
from app.Drone import Drone
from app.Camera import Camera
from app.ArucoDetector import ArucoDetector
from cv2 import aruco

class ArucoCentralizer:
    def __init__(self, drone: Drone, camera: Camera):
        self.INTEREST_REGION_PIXELS = 25
        self.GREEN = (0, 255, 0)
        self.RED = (0, 0, 255)
        self.MIN_COUNT = 20
        self.drone = drone
        self.camera = camera

    def detect_arucos(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        centers = []
        if ids is not None:
            for corner in corners:
                x = int(sum([c[0] for c in corner[0]]) / 4)
                y = int(sum([c[1] for c in corner[0]]) / 4)
                centers.append((x, y))
                
        return ids, corners, centers
    
    
    def calculate_offset(self, center, image_shape):
        image_center_x, image_center_y = image_shape[1] // 2, image_shape[0] // 2
        offset_x = center[0] - image_center_x
        offset_y = center[1] - image_center_y
        return offset_x, offset_y

    def detect_and_process_arucos(self):
        ids, corners, centers = self.detect_arucos(self.camera.frame)
        return ids, centers

    def draw_reference_square(self):
        image_center_x, image_center_y = self.camera.frame.shape[1] // 2, self.camera.frame.shape[0] // 2
        cv2.rectangle(self.camera.frame, (image_center_x - self.INTEREST_REGION_PIXELS, image_center_y - self.INTEREST_REGION_PIXELS),
                      (image_center_x + self.INTEREST_REGION_PIXELS, image_center_y + self.INTEREST_REGION_PIXELS), self.GREEN, 2)

    def adjust_drone_position(self, center, offset_x, offset_y, distance_pixels, color, count):
        CONVERSION_FACTOR = 0.17 / 25
        if distance_pixels > self.INTEREST_REGION_PIXELS and count > self.MIN_COUNT:
            print('\n--------------\n')
            print(f"Distância em pixels: {distance_pixels}, Distância em metros: {distance_pixels * CONVERSION_FACTOR}")
            
            cv2.circle(self.camera.frame, center, 5, color, -1)
            
            print("ArUco não está centralizado, ajustando posição...")
            print(f"offset_x: {offset_x}, offset_y: {offset_y}...")
            
            self.drone.adjust_position(offset_x, offset_y)
            
        elif count > self.MIN_COUNT + self.MIN_COUNT:
            self.camera.cap.grab()
            
            print('ArUco Centralizado...')
            if self.drone.current_altitude() < 1.5:
                return True
            self.drone.descend(self.drone.current_altitude() - 0.5)
            print(f'Altitude: {self.drone.current_altitude()}')
            self.camera.cap.grab()
        
        return False

    def read_and_verify_capture(self):
        self.camera.read_capture()
        if not self.camera.ret:
            return False
        return True
    
    def execute(self):
        count = 0
        
        for i in range(0,200):
            print('Ajustando camera...')
            self.camera.read_capture()
            
        while True:
            self.camera.cap.grab()
              
            if not self.read_and_verify_capture():
                continue

            ids, centers = self.detect_and_process_arucos()
            self.draw_reference_square()

            if ids is not None:
                for i, center in enumerate(centers):
                    count += 1
                    offset_x, offset_y = self.calculate_offset(center, self.camera.frame.shape)
                    distance_pixels = (offset_x**2 + offset_y**2)**0.5
                    color = self.GREEN if distance_pixels <= self.INTEREST_REGION_PIXELS else self.RED
                    
                    if self.adjust_drone_position(center, offset_x, offset_y, distance_pixels, color, count):
                        return
                    break

            self.display_video()

    def display_video(self):
        cv2.imshow('Drone Camera', self.camera.frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
    