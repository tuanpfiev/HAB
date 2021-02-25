#!/bin/sh

xterm -title "GPS Logger" -hold -e "cd $HOME/HAB/GPS-Loggers; python3 GPSLoggerUART.py" &
xterm -title "Localisation" -hold -e "cd $HOME/HAB/localisation; python3 localisation_main.py" &
xterm -title "Network" -hold -e "cd $HOME/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py" &
xterm -title "Lora" -hold -e "cd $HOME/HAB/LoRa-Radio-RSSI; python3 LoraRadio1.py"
