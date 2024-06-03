from abc import ABC, abstractmethod
from pymavlink import mavutil

from app.Masks import POSITION

class DroneMoveUP(ABC):
    def __init__(self, conn) -> None:
        super().__init__()
        self.conn = conn
        
    @abstractmethod
    def execute(self, target_altitude, ascent_speed=1.0):
        raise Exception('Method not implemented')
    
class DroneTakeOff(DroneMoveUP):
    def execute(self, target_altitude, ascent_speed=1.0):
        movement_type = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        print(f"Decolando para altitude de {target_altitude} metros com velocidade de {ascent_speed} m/s...")
        self.conn.mav.command_long_send(
            self.conn.target_system, 
            self.conn.target_component, 
            movement_type, 
            0, 0, 0,
            0, 0, 0, 
            ascent_speed, target_altitude
        )
        
class DroneChangeAlt(DroneMoveUP):
    def execute(self, target_altitude, ascent_speed=5.0):
        self.conn.mav.set_position_target_local_ned_send(
            0,  # tempo boot_ms (tempo de início do boot do sistema em milissegundos)
            self.conn.target_system,  # id do sistema
            self.conn.target_component,  # id do componente
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame de referência
            POSITION,  # tipo de máscara (indica quais dos campos são ignorados)
            0, 0, -target_altitude,  # coordenadas x, y, z (em metros ou em inteiros específicos)
            0, 0, 0,  # velocidade x, y, z (em m/s)
            0, 0, 0,  # acelerações x, y, z (em m/s^2)
            0, 0  # yaw, yaw_rate (em radianos)
        )
        
class DroneMoveUPFactory:
    @staticmethod
    def create(current_alt, conn) -> DroneMoveUP: 
        type = 'takeoff' if current_alt < 1 else 'change_alt'
        
        moves = {
            'takeoff' : DroneTakeOff,
            'change_alt': DroneChangeAlt,
        }
           
        return moves[type](conn)