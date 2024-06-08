import cv2
import numpy as np
from cv2 import aruco
import math

from app.Camera import Camera
from app.Drone import Drone

class ArucoCentralizer:
    def __init__(self, drone: Drone, camera: Camera):
        self.marker_size = 15
        self.INTEREST_REGION_PIXELS = 25
        self.GREEN = (0, 255, 0)
        self.RED = (0, 0, 255)
        self.MIN_COUNT = 5
        self.drone = drone
        self.count = 0
        self.camera = camera

        # Carregar matriz de calibração da câmera e distorção
        calib_path = ""
        self.camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
        self.camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')
        
        # Matriz de rotação de 180 graus ao redor do eixo x
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0

    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        assert (self.isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

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
        return ids, corners, centers

    def draw_reference_square(self):
        image_center_x, image_center_y = self.camera.frame.shape[1] // 2, self.camera.frame.shape[0] // 2
        cv2.rectangle(self.camera.frame, (image_center_x - self.INTEREST_REGION_PIXELS, image_center_y - self.INTEREST_REGION_PIXELS),
                      (image_center_x + self.INTEREST_REGION_PIXELS, image_center_y + self.INTEREST_REGION_PIXELS), self.GREEN, 2)

    def adjust_drone_position(self, center, offset_x, offset_y, distance_pixels, color):
        CONVERSION_FACTOR = 0.17 / 25
        print(self.count)
        if distance_pixels > self.INTEREST_REGION_PIXELS and self.count > self.MIN_COUNT:
            print('\n--------------\n')
            print(f"Distância em pixels: {distance_pixels}, Distância em metros: {distance_pixels * CONVERSION_FACTOR}")
            cv2.circle(self.camera.frame, center, 5, color, -1)
            print("ArUco não está centralizado, ajustando posição...")
            print(f"offset_x: {offset_x}, offset_y: {offset_y}...")
            self.drone.adjust_position(offset_x, offset_y)
            self.count = 0
        elif self.count > 25:
            print('ArUco Centralizado...')
            return True
        return False

    def read_and_verify_capture(self):
        self.camera.read_capture()
        return self.camera.ret

    def execute(self):
        for i in range(0, 200):
            print('Ajustando câmera...')
            self.camera.read_capture()
        while True:
            self.camera.cap.grab()
            if not self.read_and_verify_capture():
                continue

            ids, corners, centers = self.detect_and_process_arucos()
            self.draw_reference_square()

            if ids is not None:
                for i, center in enumerate(centers):
                    self.count += 1
                    offset_x, offset_y = self.calculate_offset(center, self.camera.frame.shape)
                    distance_pixels = (offset_x**2 + offset_y**2)**0.5
                    color = self.GREEN if distance_pixels <= self.INTEREST_REGION_PIXELS else self.RED

                    # Adicionar cálculo da posição da câmera em relação ao marcador
                    rvec, tvec = self.calculate_pose(corners[i])
                    self.display_pose_info(rvec, tvec)

                    if self.adjust_drone_position(center, offset_x, offset_y, distance_pixels, color):
                        return
                    break

            self.display_video()

    def calculate_pose(self, corner):
        ret = aruco.estimatePoseSingleMarkers([corner], self.marker_size, self.camera_matrix, self.camera_distortion)
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
        return rvec, tvec

    def display_pose_info(self, rvec, tvec):
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        # Posição do marcador em relação à câmera
        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
        cv2.putText(self.camera.frame, str_position, (0, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Atitude do marcador em relação à câmera
        roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(self.R_flip * R_tc)
        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (math.degrees(roll_marker), math.degrees(pitch_marker),
                                                                     math.degrees(yaw_marker))
        cv2.putText(self.camera.frame, str_attitude, (0, 150), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Posição da câmera em relação ao marcador
        pos_camera = -R_tc * np.matrix(tvec).T
        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2])
        cv2.putText(self.camera.frame, str_position, (0, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Atitude da câmera em relação ao marcador
        roll_camera, pitch_camera, yaw_camera = self.rotationMatrixToEulerAngles(self.R_flip * R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (math.degrees(roll_camera), math.degrees(pitch_camera),
                                                                     math.degrees(yaw_camera))
        cv2.putText(self.camera.frame, str_attitude, (0, 250), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, cv2.LINE_AA)

    def display_video(self):
        cv2.imshow('Drone Camera', self.camera.frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
