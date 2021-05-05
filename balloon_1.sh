#!/bin/sh
# run this shell script as: ./balloon.sh 1 or ./balloon.sh 2, where 1 and 2 are system IDs
echo "Running SYSTEM" $1
#if [$1 == "0"]
#then  
#	xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkGS.py"
#else
	xterm -geometry 90x30+200+00 -title "IMU" -hold -e "cd ~/HAB/XSENS_Test/build; ./XSENS_Test" &
	sleep 1; xterm -geometry 90x30+200+450 -title "Thermal log" -hold -e "cd ~/HAB/Thermal_Monitor; python3 jetson_temp_monitor.py $1" &
	sleep 4; xterm -geometry 90x30+1400+450 -title "Lora1" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait /dev/ttyUSB0 5100" &
	sleep 4.1; xterm -geometry 90x30+1400+450 -title "Lora2" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait /dev/ttyUSB1 5110" &
	sleep 5; xterm -geometry 90x30+800+0 -title "GPS Logger" -hold -e "cd ~/HAB/GPS-Loggers-old; python3 GPSLoggerUART.py /dev/ttyUSB2" &
	sleep 6; xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py $1"&
	sleep 8; xterm -geometry 90x30+800+450 -title "Localisation" -hold -e "cd ~/HAB/localisation-RSSI; python3 localisation_main.py"&
#&			
	sleep 8.9; xterm -title "EKF" -hold -e "cd ~/HAB/EKF; python3 main_EKF.py $1"
#fi
	

