from app.Drone import Drone 
from app.ArucoDetector import ArucoDetector
from app.Camera import Camera
from app.FileManager import FileManager

def capture_images_and_metadata(
    drone: Drone, 
    cameras: dict, 
    aruco_detector: ArucoDetector, 
    fileManager: FileManager, 
    alt):
    
    for camera_type, camera in cameras.items():
        try:
            camera = Camera()
            fileManager.create_type_dir(camera_type)
            
            camera.initialize_video_capture(camera_type)
            print(f"Iniciando captura de imagens da câmera tipo: {camera_type}")
        
            camera.capture_images_and_metadata(drone, fileManager, aruco_detector, alt)
        except Exception as e:
            print('Error: ' + str(e))
            print('Erro em capturar imagem')

def capture_images_in_alts(
    drone: Drone, 
    fileManger: FileManager, 
    aruco_detector: ArucoDetector, 
    cameras: dict):
    
    for i in range(1,6):
        
        HEIGHT = (i*3) 
        print(f"Altitude definida para {HEIGHT} METROS")
        drone.ascend(HEIGHT)
        capture_images_and_metadata(
            drone, cameras, aruco_detector, fileManger, HEIGHT
        )

def main():
    # Instancia o drone, a câmera e o detector de ArUco
    drone = Drone()
    aruco_detector = ArucoDetector()
    fileManger = FileManager()
    
    if not drone.connected():
        print("Falha ao conectar com o drone.")
        return
    
     # Defina os tipos de câmera que você possui
    camera_types = ['imx', 'rtsp', 'analog', 'esp32']
    cameras = {type: Camera() for type in camera_types}

    fileManger.create_base_dirs()

    try:
        drone.change_to_guided_mode()

        drone.arm_drone()
       
        capture_images_in_alts(drone, fileManger, aruco_detector, cameras)
       
        drone.land()
        drone.disarm()
    except:
        drone.land()
        drone.disarm()
    finally:
        drone.conn.close()

if __name__ == "__main__":
    main()
