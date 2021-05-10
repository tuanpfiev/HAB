#!/bin/bash
# run this shell script as: ./balloon.sh 1 or ./balloon.sh 2, where 1 and 2 are system IDs
echo "Running SYSTEM" $1

allBalloon=(1 2 3)
currentBalloon=$1
neighborBalloon=(${allBalloon[@]/$currentBalloon})

#echo ${neighborBalloon[1]}	

	#xterm -geometry 90x30+200+00 -title "IMU" -hold -e "cd ~/HAB/XSENS_Test/build; ./XSENS_Test" &
	#sleep 1; xterm -geometry 90x30+200+450 -title "Thermal log" -hold -e "cd ~/HAB/Thermal_Monitor; python3 jetson_temp_monitor.py $1" &
	#sleep 1.1; xterm -title "TemperatureSensor" -hold -e "cd ~/HAB/temperatureSensor; python3 temperatureSensor.py $1" &
	#sleep 2; xterm -geometry 90x30+800+0 -title "GPS Logger" -hold -e "cd ~/HAB/GPS-Loggers-old; python3 GPSLoggerUART.py /dev/ttyUSB2" &
	#sleep 3; xterm -geometry 90x30+1400+0 -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py $1"&
	#sleep 4; xterm -geometry 90x30+1400+450 -title "Lora1" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py start /dev/ttyUSB0 ${neighborBalloon[0]} $1" &
	#sleep 4.1; xterm -geometry 90x30+1400+450 -title "Lora2" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait /dev/ttyUSB1 ${neighborBalloon[1]} $1" &
	
	#sleep 8; xterm -geometry 90x30+800+450 -title "Localisation" -hold -e "cd ~/HAB/localisation-RSSI; python3 localisation_main.py"
#&			
	#sleep 8.9; xterm -title "EKF" -hold -e "cd ~/HAB/EKF-3Balloons; python3 main_EKF.py $1"
#fi
	xterm  -title "IMU" -hold -e "cd ~/HAB/XSENS_Test/build; ./XSENS_Test" &
	sleep 1; xterm  -title "Thermal log" -hold -e "cd ~/HAB/Thermal_Monitor; python3 jetson_temp_monitor.py $1" &
	sleep 1.1; xterm -title "TemperatureSensor" -hold -e "cd ~/HAB/temperatureSensor; python3 temperatureSensor.py $1" &
	sleep 2; xterm  -title "GPS Logger" -hold -e "cd ~/HAB/GPS-Loggers-old; python3 GPSLoggerUART.py /dev/ttyUSB2" &
	sleep 3; xterm  -title "Network" -hold -e "cd ~/HAB/Simple-RFD900-Network; python3 RadioNetworkMain.py $1"&
	sleep 4; xterm  -title "Lora1" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py start /dev/ttyUSB0 ${neighborBalloon[0]} $1" &
	sleep 4.1; xterm  -title "Lora2" -hold -e "cd ~/HAB/LoRa-Radio-RSSI; python3 LoraRadio.py wait /dev/ttyUSB1 ${neighborBalloon[1]} $1" &
	
	sleep 7; xterm -title "Localisation" -hold -e "cd ~/HAB/localisation-RSSI; python3 localisation_main.py" &
#&			
	sleep 6; xterm -title "EKF" -hold -e "cd ~/HAB/EKF-3Balloons; python3 main_EKF.py $1"

