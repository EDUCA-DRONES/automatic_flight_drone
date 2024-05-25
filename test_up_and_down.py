from app.Drone import Drone
import time

def main():
    drone = Drone()
    
    if not drone.connected():
        print("Falha ao conectar com o drone.")
        return

    drone.solicit_telemetry()
    drone.change_to_guided_mode()
    drone.arm_drone()
    drone.takeoff(3)
    
    drone.move_north(10)
                  
    # drone.land()
    # drone.disarm()
       
        
if __name__ == "__main__":
    main()
