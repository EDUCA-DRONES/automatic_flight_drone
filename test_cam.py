from app.Drone import Drone 
from app.ArucoDetector import ArucoDetector
from app.Camera import Camera
from app.FileManager import FileManager

drone = Drone()
aruco_detector = ArucoDetector()
fileManager = FileManager()

camera_types = ['imx', 'rtsp', 'analog', 'esp32']
cameras = {type: Camera() for type in camera_types}

def main():
    
    if not drone.connected():
        print("Falha ao conectar com o drone.")
        return ''

    for camera_type, camera in cameras.items():
            try:
                camera = Camera()
                fileManager.create_type_dir(camera_type)
                
                camera.initialize_video_capture(camera_type)
                print(f"Iniciando captura de imagens da câmera tipo: {camera_type}")
            
                camera.capture_images_and_metadata(drone, fileManager, aruco_detector, 9)
            except Exception as e:
                print(e)
                print('Erro em capturar imagem')

main()