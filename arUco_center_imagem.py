import cv2
import cv2.aruco as aruco
import numpy as np
import time
from pymavlink import mavutil
import os
from datetime import datetime

def detect_arucos(image):
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


def calculate_offset(center, image_shape):
    image_center_x, image_center_y = image_shape[1] // 2, image_shape[0] // 2
    offset_x = center[0] - image_center_x
    offset_y = center[1] - image_center_y
    return offset_x, offset_y


def adjust_drone_position(master, offset_x, offset_y, sensitivity=0.1):
    move_x = -offset_x * sensitivity
    move_y = -offset_y * sensitivity
    print(f"Ajustando posição: move_x: {move_x}, move_y: {move_y}")

    # Envie o comando para o drone
    master.mav.set_position_target_local_ned_send(
        time_boot_ms=0,
        target_system=master.target_system,
        target_component=master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=0b0000111111000111,  # Considera apenas velocidades
        x=0, y=0, z=0,
        vx=move_x, vy=move_y, vz=0,
        afx=0, afy=0, afz=0,
        yaw=0, yaw_rate=0)

def calculate_and_adjust_drone_position(master, center, image_shape, sensitivity=0.1):
    image_center_x, image_center_y = image_shape[1] // 2, image_shape[0] // 2
    offset_x = center[0] - image_center_x
    offset_y = center[1] - image_center_y
    distance = (offset_x**2 + offset_y**2)**0.5  # Distância euclidiana em pixels

    print(f"Pixel offset: ({offset_x}, {offset_y}), Distance: {distance} pixels")

    # Converte distância de pixels para metros (100 pixels = 1 metro)
    distance_meters = distance / 100

    # Movimentação do drone baseada na distância em metros
    move_x = -offset_x * sensitivity
    move_y = -offset_y * sensitivity

    # Enviar o comando para o drone (usando distance_meters para possíveis ajustes adicionais)
    master.mav.set_position_target_local_ned_send(
        time_boot_ms=0,
        target_system=master.target_system,
        target_component=master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=0b0000111111000111,  # Considera apenas velocidades
        x=0, y=0, z=0,
        vx=move_x, vy=move_y, vz=0,
        afx=0, afy=0, afz=0,
        yaw=0, yaw_rate=0)
    print(f"Commanding drone to move: vx={move_x}, vy={move_y}")


"""def main_loop(cap, master):
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        ids, corners, centers = detect_arucos(frame)
        if ids is not None:
            for i, center in enumerate(centers):
                calculate_and_adjust_drone_position(master, center, frame.shape)
                color = (0, 0, 255)  # Vermelho por padrão
                if abs(offset_x) <= 10 and abs(offset_y) <= 10:
                    color = (0, 255, 0)  # Verde se estiver no centro
                cv2.circle(frame, center, 5, color, -1)
                print(f"ArUco {ids[i]} center at {center}, offset_x: {offset_x}, offset_y: {offset_y}")
                if abs(offset_x) > 10 or abs(offset_y) > 10:
                    adjust_drone_position(master, offset_x, offset_y)
                # Desenha retângulo de região central
                cv2.rectangle(frame, (cx - 10, cy - 10), (cx + 10, cy + 10), (255, 255, 0), 1)
             # Mostra o vídeo com um retângulo central para referência
            cv2.rectangle(frame, (image_center_x - 200, image_center_y - 200), 
                      (image_center_x + 200, image_center_y + 200), (255, 255, 0), 2)
        # Mostra o vídeo
        cv2.imshow('Drone Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
"""
"""def main_loop(cap, master):
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Define o centro da imagem
        image_center_x, image_center_y = frame.shape[1] // 2, frame.shape[0] // 2

        # Desenha um retângulo central na imagem para referência
        cv2.rectangle(frame, (image_center_x - 200, image_center_y - 200),
                      (image_center_x + 200, image_center_y + 200), (255, 255, 0), 2)  # Amarelo

        ids, corners, centers = detect_arucos(frame)
        if ids is not None:
            for i, center in enumerate(centers):
                offset_x, offset_y = calculate_offset(center, frame.shape)
                color = (0, 0, 255)  # Vermelho por padrão
                if abs(offset_x) <= 200 and abs(offset_y) <= 200:
                    color = (0, 255, 0)  # Verde se estiver dentro da região central

                # Desenha um círculo no centro do ArUco
                cv2.circle(frame, center, 5, color, -1)
                # Imprime informações sobre o deslocamento e a distância
                print(f"ArUco {ids[i]} center at {center}, offset_x: {offset_x}, offset_y: {offset_y}")
                # Ajusta a posição do drone baseada nos offsets
                adjust_drone_position(master, offset_x, offset_y)

        # Mostra o vídeo
        cv2.imshow('Drone Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
"""

def main_loop(cap, master):
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Define o centro da imagem
        image_center_x, image_center_y = frame.shape[1] // 2, frame.shape[0] // 2

        # Desenha um quadrado verde pequeno no centro da imagem para referência
        cv2.rectangle(frame, (image_center_x - 10, image_center_y - 10),
                      (image_center_x + 10, image_center_y + 10), (0, 255, 0), 2)  # Verde

        ids, corners, centers = detect_arucos(frame)
        if ids is not None:
            for i, center in enumerate(centers):
                offset_x, offset_y = calculate_offset(center, frame.shape)
                distance_pixels = (offset_x**2 + offset_y**2)**0.5
                color = (0, 255, 0) if distance_pixels <= 10 else (0, 0, 255)  # Verde se <= 1m, senão vermelho
                #color = (0, 0, 255)  # Vermelho por padrão
                print(f"Distância em pixels: {distance_pixels}, Distância em metros: {distance_pixels / 100}")
                
                # Desenha um círculo no centro do ArUco
                cv2.circle(frame, center, 5, color, -1)

                # Verifica se o ArUco está dentro do quadrado verde central
                #if abs(offset_x) <= 10 and abs(offset_y) <= 10:
                  #  color = (0, 255, 0)  # Verde se estiver dentro
                 #   print("ArUco está centralizado.")
                if distance_pixels > 100:
                    print("ArUco não está centralizado, ajustando posição...")
                    adjust_drone_position(master, offset_x, offset_y)

                

                # Imprime informações sobre o deslocamento e a distância
                print(f"ArUco {ids[i]} center at {center}, offset_x: {offset_x}, offset_y: {offset_y}")

        # Mostra o vídeo
        cv2.imshow('Drone Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def initialize_video_capture(source):
    print("Inicializando captura de vídeo...")
    return cv2.VideoCapture(int(source) if source.isdigit() else source)

if __name__ == "__main__":
    # Conexão inicial com o drone via protocolo UDP. Versão simulada com mavproxy e dronekit-sitl
    master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
    # Checagem se a conexão com o drone foi estabelecida
    if master.wait_heartbeat(timeout=5):
        print("Conexão estabelecida com sucesso.")
    else:
        print("Falha ao conectar com o drone.")
    

    cap = initialize_video_capture('0')
    
    main_loop(cap, master)
