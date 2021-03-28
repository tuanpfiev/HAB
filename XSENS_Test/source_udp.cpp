#include <iostream>
#include <ostream>
#include <fstream>
#include <iomanip>
#include <list>
#include <string>
#include <chrono>
#include <system_error>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include "xsens.h"

Journaller* gJournal = 0;

int main(int argc,char **argv)
{
	int system_id;

	try
	{
		system_id = std::stoi(argv[1]);
	}
	catch(const std::exception& e)
	{
		system_id = 1;
	}

	int udpSocket, nBytes;
	
	struct sockaddr_in serverAddr, clientAddr;
	socklen_t addr_size, client_addr_size;
	int i;

	udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(5003);
	serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));

	// long rc = bind(udpSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr));

	// std::cout << rc << std::endl;
	
    std::cout << "Creating XsControl object..." << std::endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](std::string errorString)
	{
		control->destruct();
		std::cout << errorString << std::endl;
		std::cout << "Press [ENTER] to continue." << std::endl;
		std::cin.get();
		return -1;
	};

	std::cout << "Scanning for devices..." << std::endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	std::cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

	std::cout << "Opening port..." << std::endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);

	std::cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << std::endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	std::cout << "Putting device into configuration mode..." << std::endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	std::cout << "Configuring the device..." << std::endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
	configArray.push_back(XsOutputConfiguration(XDI_EulerAngles, 0));
	configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 0));
	configArray.push_back(XsOutputConfiguration(XDI_FreeAcceleration, 0));
	configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 0));
	configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	std::cout << "Putting device into measurement mode..." << std::endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	std::cout << "Starting recording..." << std::endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");

	std::cout << "\nMain loop. Recording data." << std::endl;
	std::cout << std::string(79, '-') << std::endl;

	std::ofstream big_effoff_csv;
	big_effoff_csv.open("imu_data_file.csv");

	big_effoff_csv << "system_id,";
	big_effoff_csv << "m_epoch,";
	big_effoff_csv << "r_acc_x,";
	big_effoff_csv << "r_acc_y,";
	big_effoff_csv << "r_acc_z,";
	big_effoff_csv << "gyr_x,";
	big_effoff_csv << "gyr_y,";
	big_effoff_csv << "gyr_z,";
	big_effoff_csv << "mag_x,";
	big_effoff_csv << "mag_y,";
	big_effoff_csv << "mag_z,";
	big_effoff_csv << "q_w,";
	big_effoff_csv << "q_i,";
	big_effoff_csv << "q_j,";
	big_effoff_csv << "q_k,";
	big_effoff_csv << "roll,";
	big_effoff_csv << "pitch,";
	big_effoff_csv << "yaw\n";

	big_effoff_csv.close();

	std::ofstream trn_csv;
	trn_csv.open("/home/rmit/HAB-ACTAN/providence/cmu331s4_acquisition/acquisitions/imu_log.csv");
	
	trn_csv << "high_res_epoch,";
	trn_csv << "q_w,";
	trn_csv << "q_i,";
	trn_csv << "q_j,";
	trn_csv << "q_k,";
	trn_csv << "r_acc_x,";
	trn_csv << "r_acc_y,";
	trn_csv << "r_acc_z\n";

	trn_csv.close();
	
	bool recorded;

	while (true)
	{
		std::ofstream data;
		data.open("imu_data_file.csv", std::ios::app);
		
		std::ofstream trn_data;
		trn_data.open("/home/rmit/HAB-ACTAN/providence/cmu331s4_acquisition/acquisitions/imu_log.csv", std::ios::app);

		recorded = false;

		while (!recorded)
		{
			if (callback.packetAvailable())
			{
				XsDataPacket packet = callback.getNextPacket();

				if (packet.containsCalibratedAcceleration() && packet.containsCalibratedGyroscopeData() && packet.containsCalibratedMagneticField() && packet.containsOrientation())
				{
					std::string data_packet = "{|SYSTEM_ID: " + std::to_string(system_id) + "; "; 
					data << system_id << ",";
					data << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ",";
					data_packet += "EPOCH: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()) + "; ";

					XsVector r_acc = packet.calibratedAcceleration();
					data << r_acc[0] << "," << r_acc[1] << "," << r_acc[2] << ",";
					data_packet += "ACCELERATION: " + std::to_string(r_acc[0]) + "," + std::to_string(r_acc[1]) + "," + std::to_string(r_acc[2]) + "; ";

					XsVector gyr = packet.calibratedGyroscopeData();
					data << gyr[0] << "," << gyr[1] << "," << gyr[2] << ",";
					data_packet += "GYRO: " + std::to_string(gyr[0]) + "," + std::to_string(gyr[1]) + "," + std::to_string(gyr[2]) + "; ";

					XsVector mag = packet.calibratedMagneticField();
					data << mag[0] << "," << mag[1] << "," << mag[2] << ",";
					data_packet += "MAGNETIC_VECTOR: " + std::to_string(mag[0]) + "," + std::to_string(mag[1]) + "," + std::to_string(mag[2]) + "; ";

					XsQuaternion quaternion = packet.orientationQuaternion();
					data << quaternion.w() << "," << quaternion.x() << "," << quaternion.y() << "," << quaternion.z() << ",";
					auto now = std::chrono::high_resolution_clock::now();
					trn_data << std::to_string(now.time_since_epoch().count()) << "," << quaternion.w() << "," << quaternion.x() << "," << quaternion.y() << "," << quaternion.z() << "," << r_acc[0] << "," << r_acc[1] << "," << r_acc[2] << "\n";
					data_packet += "RAW_QT: " + std::to_string(quaternion.w()) + "," + std::to_string(quaternion.x()) + "," + std::to_string(quaternion.y()) + "," + std::to_string(quaternion.z()) + "; ";

					XsEuler euler = packet.orientationEuler();
					data << euler.roll() << "," << euler.pitch() << "," << euler.yaw() << "\n";
					data_packet += "EULER_321: " + std::to_string(euler.roll()) + "," + std::to_string(euler.pitch()) + "," + std::to_string(euler.yaw()) + ";}";

					sendto(udpSocket, data_packet.c_str(), strlen(data_packet.c_str()), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
					std::cout << "ACCELERATION: " + std::to_string(r_acc[0]) + ", " + std::to_string(r_acc[1]) + ", " + std::to_string(r_acc[2]) + ", ===GYRO: " + std::to_string(gyr[0]) + ", " + std::to_string(gyr[1]) + ", " + std::to_string(gyr[2]) + " ===EULER_321: " + std::to_string(euler.roll()) + ", " + std::to_string(euler.pitch()) + ", " + std::to_string(euler.yaw()) <<std::endl;
					recorded = true;
					XsTime::msleep(200);
				}
			}
		}

		data.close();
		trn_data.close();
	}
	std::cout << "\n" << std::string(79, '-') << "\n";
	std::cout << std::endl;

	std::cout << "Stopping recording..." << std::endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	std::cout << "Closing port..." << std::endl;
	control->closePort(mtPort.portName().toStdString());

	std::cout << "Freeing XsControl object..." << std::endl;
	control->destruct();

	std::cout << "Successful exit." << std::endl;

	return 0;
}
