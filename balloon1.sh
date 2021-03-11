#!/bin/sh

xterm -geometry 90x30+800+0 -title "GPS Logger" -hold -e "cd ~/HAB/GPS-Loggers; python3 GPSLoggerUART.py" &
xterm -geometry 90x30+800+450 -title "Localisation" -hold -e "cd ~/HAB/localisation; python3 localisation_main.py"&
xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py"&
xterm -geometry 90x30+1400+450 -title "Lora" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait"
