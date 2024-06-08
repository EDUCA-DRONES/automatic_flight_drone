import numpy as np
import cv2
import glob
from pymavlink import mavutil
import pickle
import time

# Step 1: Camera Calibration

# Define the chessboard size (number of internal corners per row and column)
chessboard_size = (7, 7)

# Prepare object points (0,0,0), (1,0,0), (2,0,0), ..., (7,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# List of calibration images
images = glob.glob('calibration_imgs/*.jpg')

# Iterate through the list and search for chessboard corners
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points and image points
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Load a test image
img = cv2.imread('calibration_imgs/2024-06-08_10-50-07.jpg')
img_size = (img.shape[1], img.shape[0])

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

# Undistort a test image and save the result
dst = cv2.undistort(img, mtx, dist, None, mtx)
cv2.imwrite('calibration_imgs/test_undist.jpg', dst)

# Save calibration results for future use
dist_pickle = {"mtx": mtx, "dist": dist}
with open("calibration_imgs/wide_dist_pickle.p", "wb") as f:
    pickle.dump(dist_pickle, f)

# Step 2: ArUco Detection and Precision Landing

# Load calibration results
with open("calibration_imgs/wide_dist_pickle.p", "rb") as f:
    calib_data = pickle.load(f)
camera_matrix = calib_data["mtx"]
dist_coeffs = calib_data["dist"]

# Setup ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)  
parameters = cv2.aruco.DetectorParameters()
# Initialize the camera
cap = cv2.VideoCapture('rtsp://admin:admin@192.168.0.110:554/11')  # Change to 1 if using an external camera

# Connect to the drone
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to the drone!")

def send_land_message(x, y):
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),  # type_mask (only positions enabled)
            x, y, 0,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0))  # yaw, yaw_rate

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.15, camera_matrix, dist_coeffs)  # 0.15 is the marker length in meters
        
        for i in range(len(ids)):
            # Draw axis and marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            
            # Send landing message to drone
            x, y, z = tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2]
            send_land_message(x, y)
            print(f"Marker ID: {ids[i]}, Position: {tvecs[i]}")

   
    height, width, _ = frame.shape
    cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 0, 255), 2)
    cv2.line(frame, (0, height // 2), (width, height // 2), (0, 0, 255), 2)
    cv2.circle(frame, (0, 0), 5, (0, 0, 255), -1)

    p = 4 * 0.15
    a = 0.15 * 0.15
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,f'Perimetro: {p}',(10,500), font, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,f'Area: {a}',(10,200), font, 4,(255,255,255),2,cv2.LINE_AA)
      
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
