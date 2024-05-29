import cv2
import os
from datetime import datetime
import time
from pymavlink import mavutil
from app.Drone import Drone 
from app.ArucoDetector import ArucoDetector
from app.Camera import Camera


"""def capture_images_and_metadata(drone, camera, save_path, meta_path, type_camera, num_images=3, pause_time=3):
    for i in range(num_images):
        time.sleep(pause_time)  # Espera entre capturas

        # Obter a localização atual do drone
        latitude, longitude, altitude = drone.get_gps_position()

        #ret, frame = cap.read()
        camera.read_capture()
        if camera.ret:
            timestamp = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
            image_filename = f"{save_path}/camera_{type_camera}_Image_{i+1}_{timestamp}.jpg"
            cv2.imwrite(image_filename, camera.frame)
            print(f"Imagem {i+1} capturada da câmera {type_camera} e salva em: {image_filename}")

            # Salvar metadados
            metadata_filename = f"{meta_path}/camera_{type_camera}_Metadata_{i+1}_{timestamp}.txt"
            with open(metadata_filename, 'w') as metafile:
                metafile.write(f"Timestamp: {timestamp}\n")
                metafile.write(f"Latitude: {latitude}\n")
                metafile.write(f"Longitude: {longitude}\n")
                metafile.write(f"Altitude: {altitude}m\n")"""

def capture_images_and_metadata(drone, cameras, save_path, meta_path, aruco_detector, num_images, pause_time):
    for camera_type, camera in cameras.items():
        camera.initialize_video_capture(camera_type)
        print(f"Iniciando captura de imagens da câmera tipo: {camera_type}")
        for i in range(num_images):
            time.sleep(pause_time)  # Espera entre capturas

            # Obter a localização atual do drone
            latitude, longitude, altitude = drone.get_gps_position()

            camera.read_capture()
            if camera.ret:
                processed_image, ids, corners = aruco_detector.detect_arucos(camera.frame)

                timestamp = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
                image_filename = f"{save_path}/camera_{camera_type}_Image_{i+1}_{timestamp}.jpg"
                cv2.imwrite(image_filename, processed_image)
                print(f"Imagem {i+1} capturada da câmera {camera_type} e salva em: {image_filename}")

                # Salvar metadados
                metadata_filename = f"{meta_path}/camera_{camera_type}_Metadata_{i+1}_{timestamp}.txt"
                with open(metadata_filename, 'w') as metafile:
                    metafile.write(f"Timestamp: {timestamp}\n")
                    metafile.write(f"Latitude: {latitude}\n")
                    metafile.write(f"Longitude: {longitude}\n")
                    metafile.write(f"Altitude: {altitude}m\n")
            
            for _ in range(40):  # Limpa os últimos 88 frames para garantir frescor
                camera.cap.grab()


def main():
    # Instancia o drone, a câmera e o detector de ArUco
    drone = Drone()
    if not drone.connected():
        print("Falha ao conectar com o drone.")
        return
    
     # Defina os tipos de câmera que você possui
    camera_types = ['computer', 'imx', 'analog']
    cameras = {type: Camera() for type in camera_types}

    #camera = Camera()
    aruco_detector = ArucoDetector()


    #camera.initialize_video_capture('computer') # ou 'rtsp' se estiver usando uma câmera RTSP
    save_path = 'captured_images_tests'
    meta_path = 'metadata_images_tests'
    os.makedirs(save_path, exist_ok=True)
    os.makedirs(meta_path, exist_ok=True)

    try:
        # Muda para o modo GUIDED
        drone.change_to_guided_mode()

        # Arma o drone
        drone.arm_drone()

        # Define a altitude desejada
        target_altitude = 3  # em metros
        print("Altitude definida: 3 METROS")

        # Decola para a altitude definida
        drone.takeoff(target_altitude)
        

        # Capturar imagens e metadados para cada tipo de câmera
        capture_images_and_metadata(drone, cameras, save_path, meta_path, aruco_detector, num_images=3, pause_time=1.5)

        """hover_time = 60
        start_time = time.time()

        while time.time() - start_time < hover_time:
            for camera in cameras.values():
                camera.read_capture()
                if camera.ret:
                    # Processamento de imagem aqui
                    # Detecta marcadores ArUco na imagem
                    aruco_detector.detect_arucos(camera.frame)"""

        # Pousa o drone após o tempo de voo
        drone.land()
        # Desarma o drone após o pouso
        drone.disarm()
    except KeyboardInterrupt:
        drone.land()
        # Desarma o drone após o pouso
        drone.disarm()
    finally:
        pass
        #camera.cap.realese()  # Libera o recurso da câmera, caso esteja usando 'Camera' com 'cv2.VideoCapture'


if __name__ == "__main__":
    main()
