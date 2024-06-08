from geopy.distance import distance
from geopy.point import Point
from pymavlink import mavutil

class GPS: 
    def __init__(self) -> None:
        self.directions_dict = {
            'north': 0,
            'east': 90,
            'south': 180,
            'west': 270
        }
        

    def calculate_coord(self, lat_initital, long_inital, meters, direction):
        """
        Calcula a nova coordenada após deslocamento de uma distância em meters em uma direção específica.

        :param lat_initital: Latitude inicial
        :param long_inital: Longitude inicial
        :param meters: Distância em meters para deslocamento
        :param direction: Direção do deslocamento em graus (north, east, south, west)
        :return: (new_lat, new_long)
        """
       
        ponto_inicial = Point(lat_initital, long_inital)
        new_distance = distance(meters=meters)
        new_point = new_distance.destination(point=ponto_inicial, bearing=self.directions_dict[direction])
        
        new_lat = new_point.latitude
        new_long = new_point.longitude
        
        return new_lat, new_long

    
# # Exemplo de uso
# lat_initital = -14.303033
# long_inital = -42.6944275  
# meters = 6
# directions = 'east'  # Norte

# gps = GPS()
# new_lat, new_long = gps.calculate_coord(lat_initital, long_inital, meters, directions)
# print(f"Nova latitude: {new_lat}, Nova longitude: {new_long}")