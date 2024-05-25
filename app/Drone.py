from math import cos, radians
from pymavlink import mavutil
import time 


class DroneConfig:
    def __init__(self) -> None:
        self.GUIDED_MODE = 4
        
class Drone:
    def __init__(self) -> None:
        self.IP = '127.0.0.1'
        self.URL = f'udpin:{self.IP}:14551'
        self.METER_CONVERTER = 1000.0
        self.conn =  mavutil.mavlink_connection(self.URL)
        self.config = DroneConfig()
        self.velocity = 30
        
    def connected(self):
        return self.conn.wait_heartbeat(timeout=5)
    
    def solicit_telemetry(self):
        self.conn.mav.request_data_stream_send(self.conn.target_system, self.conn.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        
    def current_altitude(self):
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        return msg.relative_alt / self.METER_CONVERTER   # Convertendo de mm para metros

    def arm_drone(self):
        print("Armando drone...")
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = False
        while not ack:
            msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
            ack = msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0
        print("Drone armado.")
    
    def takeoff(self, altitude):
        print(f"Decolando para altitude de {altitude} metros...")
        
        self.conn.mav.command_long_send(
            self.conn.target_system, 
            self.conn.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
        )
        
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, 
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        
        while True:
            print(f"Altitude atual: {self.current_altitude()}m")

            if self.current_altitude() >= altitude * 0.95: 
                print("Altitude alvo alcançada.")
                break
    
    def land(self):
        print("Comando de pouso enviado ao drone.")
        
        self.conn.mav.command_long_send(
            self.conn.target_system, 
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        while True:
            print(f"Altura {self.current_altitude()}m")
            print('Descendo... \n')
            if self.current_altitude() < 1:
                break; 
            
        print('Desceu com crtz')
        
    
    def set_mode(self, mode):
        if mode not in self.conn.mode_mapping():
            print("Modo desconhecido:", mode)
            print("Modos disponíveis:", list(self.conn.mode_mapping().keys()))
            return

        mode_id = self.conn.mode_mapping()[mode]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        while True:
            ack = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = mavutil.mavlink.enums['MAV_RESULT'][ack.result].description
            print("Modo de comando:", ack_msg)
            if ack_msg == 'ACCEPTED':
                break
        print(f"Modo alterado para {mode}")


    def disarm(self):
        print("Desarmando o drone.")
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
    
    def change_to_guided_mode(self):
        print("Mudando para modo GUIDED...")

        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self.config.GUIDED_MODE  # GUIDED mode
        )

        # Aguarda confirmação de mudança de modo
        while True:
            msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
            if msg:
                print("Received msg: ", msg)
                expected_command = mavutil.mavlink.MAV_CMD_DO_SET_MODE
                expected_result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                print(f"Comando esperado: {expected_command} | Resultado: {expected_result}")
                
                # Verifica se o comando corresponde ao esperado e se foi aceito
                if msg.command == 11 and msg.result == 0:
                    print("GUIDED mode set.")
                    break
                else:
                    print("Still waiting for GUIDED mode confirmation...")
                    print(f"Received command: {msg.command} | Result: {msg.result}")
   
    def set_velocity_body(self, vx, vy, vz):
        """
        Sets the velocity of the drone in the body frame.
        """
        msg = self.conn.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.conn.send_mavlink(msg)
        self.conn.flush()

    def move_north(self, distance, velocity=1.0):
        """
        Moves the drone north by the specified distance at the specified velocity.
        """
        print(f"Moving north for {distance} meters at {velocity} m/s")
        duration = distance / velocity
        self.set_velocity_body(velocity, 0, 0)
        time.sleep(duration)
        self.set_velocity_body(0, 0, 0)
        print("Movement complete")

    def move_direction(self, north, east, down):
        """
        Moves the drone in the specified direction.
        """
        print(f"Moving NED for {north}m north, {east}m east, {down}m down")
        self.set_velocity_body(north, east, down)