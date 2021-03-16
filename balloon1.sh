#!/bin/sh
#xterm -geometry 90x30+200+00 -title "IMU" -hold -e "cd ~/HAB/XSENS_Test/build; ./XSENS_Test"&
sleep 0.5; xterm -geometry 90x30+800+0 -title "GPS Logger" -hold -e "cd ~/HAB/GPS-Loggers; python3 GPSLoggerUART.py" &
sleep 2.5; xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py"&
sleep 4; xterm -geometry 90x30+800+450 -title "Localisation" -hold -e "cd ~/HAB/localisation; python3 localisation_main.py"&
sleep 4; xterm -geometry 90x30+1400+450 -title "Lora" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait"&
