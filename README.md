# automatic_flight_drone
Repositório dedicado a construção de algoritmos voltados para a realização de missões com drones de forma aumatizada.

O arquivo 'requirements_venv.txt' apresenta as instalações necessárias, juntamente as suas respectivas versões, para funcionamento adequado do algoritmo em 'drone_automatic_flight_mission.py'

O arquivo 'drone_automatic_flight_mission.py' é uma versão inicial de algoritmo para voo automático de drone. Essa versão apresenta conexão bem sucedida com o VANT de três formas diferentes: 
    > Via protocolo TCP; 
    > Via Protocolo UDP; 
    > Via porta serial com telemetria conectada. 
    
Além disso, é possível inicializar determinados dispositivos de captura, mudar modo de voo para 'GUIDED', visando realizar a missão automática, e também para 'LAND', para o pouso do drone após reconhecimento de id ArUco. É possível também armar o drone e realizar o takeoff de forma bem sucedida, e após a missão e pousar, realizar o desarme do VANT. A missão é iniciada após o algoritmo detectar determinado id ArUco.