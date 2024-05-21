import cv2
import cv2.aruco as aruco
import numpy as np
import time
from pymavlink import mavutil
import os
from datetime import datetime

# Função para inicializar a captura de vídeo
def initialize_video_capture(source):
    print("Inicializando captura de vídeo...")
    return cv2.VideoCapture(int(source) if source.isdigit() else source)

# Função para detectar marcadores ArUco na imagem
def detect_arucos(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = aruco.DetectorParameters()
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    return ids, corners if ids is not None else (None, None)

# Função para mudar o modo de voo para GUIDED
def change_to_guided_mode(master):
    print("Mudando para modo GUIDED...")
    # Envia comando para mudar para modo GUIDED
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )

    # Aguarda confirmação de mudança de modo
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg:
            print("Received msg: ", msg)
            expected_command = mavutil.mavlink.MAV_CMD_DO_SET_MODE
            expected_result = mavutil.mavlink.MAV_RESULT_ACCEPTED
            print(f"Expected command: {expected_command} | Expected result: {expected_result}")
            
            # Verifica se o comando corresponde ao esperado e se foi aceito
            if msg.command == 11 and msg.result == 0:
                print("GUIDED mode set.")
                break
            else:
                print("Still waiting for GUIDED mode confirmation...")
                print(f"Received command: {msg.command} | Result: {msg.result}")

# Função para armar o drone e realizar a decolagem
def arm_and_takeoff(master, altitude):
    print("Armando o drone e preparando para decolagem...")
    arm_drone(master)
    takeoff(master, altitude)

# Função para armar o drone
def arm_drone(master):
    print("Armando drone...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    ack = False
    while not ack:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack = msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0
    print("Drone armado.")

# Função para realizar a decolagem até uma certa altitude
def takeoff(master, altitude):
    print(f"Decolando para altitude de {altitude} metros...")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    # Aguarda alcançar a altitude segura
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Convertendo de mm para metros
        print(f"Altitude atual: {current_altitude}m")

        if current_altitude >= altitude * 0.95:  # Chega perto da altitude alvo
            print("Altitude alvo alcançada.")
            break

# Função para comandar o pouso do drone
def land(master):
    print("Comando de pouso enviado ao drone.")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# Função para desarmar o drone
def disarm(master):
    print("Desarmando o drone.")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# Função para enviar uma lista de waypoints ao drone
def send_waypoints(master, waypoints):
    print("Limpando missões anteriores...")
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    response = master.recv_match(type='MISSION_ACK', blocking=True)
    if response:
        print(f"Confirmação de limpeza de missão recebida: {response.type}")

    for idx, wp in enumerate(waypoints):
        lat, lon, alt = wp['lat'], wp['lon'], wp['alt']
        lat_int = int(lat * 1E7)
        lon_int = int(lon * 1E7)

        # Verificação adicional para valores fora dos limites
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            print(f"Valores de latitude/longitude inválidos: lat {lat}, lon {lon}")
            continue  # Pula este waypoint se os valores estiverem fora dos limites

        print(f"Enviando waypoint {idx}: Lat {lat_int}, Lon {lon_int}, Alt {alt}")
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            idx,  # Sequência do waypoint
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame de referência
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Comando de waypoint
            0, 1,  # Current (0), Autocontinue (1)
            0, 0, 0, 0,  # Param1, Param2, Param3, Param4
            int(lat * 1E7), int(lon * 1E7), int(alt)  # x (lat), y (lon), z (alt)
        )
        print(f"Coordenadas - lat: '{int(lat * 1E7)}', lon: '{int(lon * 1E7)}', alt: '{alt}'")
        
        ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack:
            print(f"Confirmação de waypoint recebida: {ack.type} do sistema alvo {ack.target_system}, componente {ack.target_component}")
            if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Waypoint aceito.")
            else:
                print(f"Comando de missão rejeitado: {mavutil.mavlink.enums['MAV_MISSION_RESULT'][ack.type].description}")
                break
        else:
            print("Nenhuma confirmação recebida para o waypoint.")
        print("Espere 1 segundo...")
        time.sleep(1)  # Delay para processamento

    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))

def wait_for_drone_to_stabilize(master, target_x, target_altitude, tolerance=1, timeout=30):
    start_time = time.time()  # Captura o tempo de início para o timeout
    # Espera até que o drone esteja dentro da tolerância da posição desejada ou o timeout seja alcançado

    while True:
        if time.time() - start_time > timeout:
            print("Timeout reached, proceeding with image capture despite inaccuracy.")
            break

        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_x = msg.x
        current_z = msg.z # Altitude é negativa em LOCAL_POSITION_NED
        print(f"Checking position: Target X={target_x}, Current X={current_x}, Target Z={-target_altitude}, Current Z={current_z}")
        
        if abs(current_x - target_x) <= tolerance and abs(current_z + target_altitude) <= tolerance:
            print("Drone has stabilized at the target position.")
            break
        time.sleep(0.1)  # Pequena pausa para não sobrecarregar a rede

def capture_imagem_point_by_point(master, quant_loop, x_metros, seg_parado, y_metros_por_segundo, altitude_drone, camera_rtsp_url):
    cap = initialize_video_capture(camera_rtsp_url)
    
    # Criar diretórios para salvar as imagens e metadados, se ainda não existirem
    save_path = 'captured_images'
    meta_path = 'metadata'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.exists(meta_path):
        os.makedirs(meta_path)

    total_distance = 0  # Distância acumulada que o drone deve mover-se
    for quant in range(1, quant_loop + 1):
       print(f"Caminho nº {quant}")
       #time.sleep(1)
       total_distance += x_metros
       #print("total_distance: ", total_distance)

       #altitude_drone_Negativo = 0
       #altitude_drone_Negativo = altitude_drone_Negativo - altitude_drone
       
       print(f"Enviando informações de voo para o drone! \n Irá {x_metros} metros para frente a cada caminho e ficará {seg_parado} segundos parado")
       master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,
                       master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), total_distance, 0, -altitude_drone, y_metros_por_segundo, 0, 0, 0, 0, 0, 0, 0))
                  
       """master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,
                        master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        int(0b110111111000), 
                        x_metros, #X Position em metros (positivo para frente ou Norte)
                        0, #Y Position em metros (positivo para a direita ou Leste)
                        altitude_drone_Negativo, #Z Position em metros (positivo é para baixo)
                        y_metros_por_segundo,  #X velocidade em m/s (positivo para frente ou Norte)
                        0, #Y velocidade em m/s (positivo para a direita ou Leste)
                        0, #Z velocidade em m/s (positivo é para baixo)
                        0, #X aceleração em m/s/s (positivo para frente ou Norte)
                        0, #Y aceleração in m/s/s (positivo para a direita ou Leste)
                        0, #Z aceleração em m/s/s (positivo é para baixo)
                        0, #yaw or heading in radians (0 is forward or North)
                        0 #yaw rate in rad/s
                        )
                        )"""
       #print(master_msg)
       # Esperar até que o drone estabilize na posição desejada
       #wait_for_drone_to_stabilize(master, total_distance, altitude_drone)
    
       time.sleep(seg_parado)# Ex.: 5 segundos
       #Procedimento para capturar imagem

       for image_num in range(1, 4):  # Captura 3 imagens por caminho
            # Espera 3 segundos antes de cada captura
            print(f"Aguardando 3 segundos antes de capturar imagem {image_num} do caminho {quant}...")
            time.sleep(3)

            # Obter a localização atual do drone
            location_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            latitude = location_msg.lat / 1e7
            longitude = location_msg.lon / 1e7
            altitude = location_msg.alt / 1000.0

            # Limpa o buffer de frames para pegar o frame atual
            for _ in range(88):  # Limpa os últimos 5 frames para garantir frescor
                cap.grab()

            ret, frame = cap.read()
            if ret:
                timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                image_filename = f"{save_path}/{quant}_{image_num}_.jpg"
                cv2.imwrite(image_filename, frame)
                print(f"Imagem '{quant}_{image_num}_' capturada e salva como: {image_filename}")
                
                # Salvar metadados
                metadata_filename = f"{meta_path}/{quant}_{image_num}_.txt"
                with open(metadata_filename, 'w') as metafile:
                    metafile.write(f"Timestamp: {timestamp}\n")
                    metafile.write(f"Image: {image_filename}\n")
                    metafile.write(f"Path: {quant}\n")
                    metafile.write(f"Image Number: {image_num}\n")
                    metafile.write(f"Latitude: {latitude}\n")
                    metafile.write(f"Longitude: {longitude}\n")
                    metafile.write(f"Altitude: {altitude}m\n")
                
            else:
                print("Falha ao capturar imagem da câmera.")

        # Incremento na distância após cada caminho completo
       #x_metros += x_metros
       print(f"Preparando para mover para o próximo caminho, distância aumentada para {x_metros} metros.")

    cap.release()  # Libera o recurso da câmera
 
# Função principal que inicializa a conexão e o controle do drone
def main():
    # Conexão inicial com o drone via protocolo TCP
    master = mavutil.mavlink_connection('tcp:192.168.0.105:5760')

    # Conexão inicial com o drone via protocolo UDP. Versão simulada com mavproxy e dronekit-sitl
    #master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')

    # Conexão com drone via porta serial onde o dispositivo de telemetria está conectado.
    #master = mavutil.mavlink_connection(device="/dev/ttyUSB0", baud=57600 )  

    # Checagem se a conexão com o drone foi estabelecida
    if master.wait_heartbeat(timeout=5):
        print("Conexão estabelecida com sucesso.")
    else:
        print("Falha ao conectar com o drone.")
        return

    # Solicitação de atualização dos dados de telemetria
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    # Conexão via RTSP com a câmera acoplada ao drone
    camera_rtsp_url = 'rtsp://admin:admin@192.168.0.105:554/11'
    
    # Inicialização da captura de vídeo via uma câmera local
    cap = initialize_video_capture('0') # Substitua '0' por camera_rtsp_url para utilizar a câmera via RTSP
    try:
        # Loop principal para processamento de vídeo e detecção de ArUco
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            cv2.imshow('Video Feed', frame)
            ids, corners = detect_arucos(frame)
            if ids is not None:
                if 33 in ids:
                    print("ArUco 33 detectado, alterando para modo GUIDED e preparando para decolagem...")
                    change_to_guided_mode(master)
                    arm_and_takeoff(master, 10)

                    # Definição e envio de waypoints
                    """waypoints = [
                        {"lat": -14.301502, "lon": -42.690646, "alt": 15},
                        {"lat": -14.301302, "lon": -42.690446, "alt": 15},
                        {"lat": -14.301602, "lon": -42.690346, "alt": 15}
                    ]"""
                    
                    capture_imagem_point_by_point(master, 4, 6, 5, 2, 5, camera_rtsp_url)

                    while True:
                        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
                        print(msg)
                    time.sleep(1)

                elif 8 in ids:
                    print("ArUco 8 detectado, iniciando procedimentos de pouso...")
                    land(master)
                    disarm(master)
                    break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Simulação interrompida pelo usuário.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
