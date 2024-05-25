import cv2
import cv2.aruco as aruco

class ArucoDetector:
    def __init__(self) -> None:
        self.ids = []
        self.corners = []
        
    def detect_arucos(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        
        aruco_params = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        self.ids, self.corners = ids, corners if ids is not None else (None, None)
    
    