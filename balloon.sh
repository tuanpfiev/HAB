#!/bin/sh
# run this shell script as: ./balloon.sh 1 or ./balloon.sh 2, where 1 and 2 are system IDs
echo "Running SYSTEM" $1
#if [$1 == "0"]
#then  
#	xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkGS.py"
#else
	xterm -geometry 90x30+200+00 -title "IMU" -hold -e "cd ~/HAB/XSENS_Test/build; ./XSENS_Test" &
	sleep 7; xterm -geometry 90x30+1400+450 -title "Lora" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait" &
	sleep 5; xterm -geometry 90x30+800+0 -title "GPS Logger" -hold -e "cd ~/HAB/GPS-Loggers; python3 GPSLoggerUART.py" &
	sleep 6; xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py $1"&
	sleep 7; xterm -geometry 90x30+800+450 -title "Localisation" -hold -e "cd ~/HAB/localisation; python3 localisation_main.py"&	
	sleep 7.1; xterm -title "EKF" -hold -e "cd ~/HAB/EKF; python3 main_EKF.py $1"
#fi
	

