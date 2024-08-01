#!/bin/bash

source /opt/ros/iron/setup.bash

echo 'Finishing /usr/bin/python3 processes...'
killall /usr/bin/python3
killall multirobot_start.sh

# Adicionando comando para fechar a janela do Turtlesim
pkill -f turtlesim_node

