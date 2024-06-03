import cv2
from app.Drone import Drone
from app.Camera import Camera
from app.ArucoDetector import ArucoDetector
from cv2 import aruco

class ArucoCentralizer:
    def __init__(self):
        self.INTEREST_REGION_PIXELS = 25
        self.GREEN = (0, 255, 0)
        self.RED = (0, 0, 255)
        self.MIN_COUNT = 40

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

        
    def execute(self, drone: Drone, camera: Camera, aruco_detector: ArucoDetector):

        count = 0
        while True:
            camera.read_capture()
            if not camera.ret:
                continue

            # Define o centro da imagem
            image_center_x, image_center_y = camera.frame.shape[1] // 2, camera.frame.shape[0] // 2
        
            ids, corners, centers = self.detect_arucos(camera.frame)  

            # Desenha um quadrado verde pequeno no centro da imagem para referência
            cv2.rectangle(camera.frame, (image_center_x - self.INTEREST_REGION_PIXELS, image_center_y - self.INTEREST_REGION_PIXELS),
                        (image_center_x + self.INTEREST_REGION_PIXELS, image_center_y + self.INTEREST_REGION_PIXELS), self.GREEN, 2)  # Verde

            if ids is not None:
                
                for i, center in enumerate(centers):
                    count += 1
                    offset_x, offset_y = self.calculate_offset(center, camera.frame.shape)
                    distance_pixels = (offset_x**2 + offset_y**2)**0.5
                    color = self.GREEN if distance_pixels <= self.INTEREST_REGION_PIXELS else self.RED  # Verde se <= 1m, senão vermelho
                    #color = (0, 0, 255)  # Vermelho por padrão
                    CONVERSION_FACTOR = 0.17 / 25

                    # Verifica se o ArUco está dentro do quadrado verde central
                    #if abs(offset_x) <= 10 and abs(offset_y) <= 10:
                    #  color = self.GREEN  # Verde se estiver dentro
                    #   print("ArUco está centralizado.")
                    if distance_pixels > self.INTEREST_REGION_PIXELS and count > self.MIN_COUNT:
                          # Desenha um círculo no centro do ArUco
                        print(f"Distância em pixels: {distance_pixels}, Distância em metros: {distance_pixels * CONVERSION_FACTOR}")
                        cv2.circle(camera.frame, center, 5, color, -1)
                        print("ArUco não está centralizado, ajustando posição...")
                        print(f"offset_x: {offset_x}, offset_y: {offset_y}...")

                        drone.adjust_position(offset_x, offset_y)
                        
                    elif count > self.MIN_COUNT:
                        print('Centralizou')
                        return None
                        count = 0
                    
                    # Imprime informações sobre o deslocamento e a distância
                    print(f"ArUco {ids[i]} center at {center}, offset_x: {offset_x}, offset_y: {offset_y}")
                    break

            # Mostra o vídeo
            cv2.imshow('Drone Camera', camera.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    