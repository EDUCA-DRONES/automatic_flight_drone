#!/bin/bash

#dependencies of mavproxy
sudo apt-get install python-opencv python-wxgtk2.8 python-matplotlib python-numpy python-serial python-pil libwxgtk2.8-dev libpython2.7-dev

#download libs of application
pip install -r requirements_venv.txt

#mav proxy
sudo pip install MAVProxy

# dronekit
pip install dronekit-sitl -UI

# Baixar o arquivo MissionPlanner-latest.zip
curl -O https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-1.3.79.zip

# Verifica se o arquivo foi baixado com sucesso
if [ -f "MissionPlanner-1.3.79.zip" ]; then
    echo "Arquivo baixado com sucesso."
else
    echo "Erro ao baixar o arquivo."
    exit 1
fi

# Cria o diretório para descompactar o arquivo, se não existir
mkdir -p MissionPlanner

# Descompacta o arquivo
unzip MissionPlanner-latest.zip -d MissionPlanner

# Verifica se a descompactação foi bem-sucedida
if [ $? -eq 0 ]; then
    echo "Arquivo descompactado com sucesso."
else
    echo "Erro ao descompactar o arquivo."
    exit 1
fi

sudo apt install mono-devel