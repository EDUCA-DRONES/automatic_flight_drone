import time
import cv2
import cv2.aruco as aruco
from app.Drone import Drone

NORTH_ID = 27
SOUTH_ID = 14
EAST_ID = 63
WEST_ID = 48

class ArucoDetector:
    def __init__(self) -> None:
        self.ids = []
        self.corners = []
    
    def detect_arucos(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        
        aruco_params = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        ids, corners = ids, corners if ids is not None else (None, None)
        print('Valor do id: ' + str(ids))
        if ids and ids is not None:
            self.ids = ids
            self.corners = corners
            image = aruco.drawDetectedMarkers(image, corners, ids)

        return image, ids, corners
    
    def go_to_aruco_direction(self, drone: Drone, distance_m = 0.5):
        
        print(self.ids)
        if self.ids:
            print('Realizando movimento para direção')
            if NORTH_ID in self.ids:
                drone.move_north(distance_m)
            elif SOUTH_ID in self.ids:
                drone.move_south(distance_m)
            elif EAST_ID in self.ids:
                drone.move_east(distance_m)
            elif WEST_ID in self.ids:
                drone.move_west(distance_m)
            # else:
            self.ids = []
            
            # time.sleep(1)
            
