/**
 * @file main.cpp
 * @author Markus Bader
 * @brief
 * This main file allows you to start a naoqi module
 **/

//#include "akdefines.hpp" // The akdefines.hpp is located in $AK_DIR/include

#include <iostream>
#include "naoqi.h"

int main ( int argc, char *argv[] ) {


    std::string address = "192.168.1.216";

    AK::NaoQi *p = AK::NaoQi::getInstance(address);
    p->logInfo("Hallo");
    std::vector<std::string> sensors;
    std::vector<float> values;
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/AccX/Sensor/Value"));
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/AccY/Sensor/Value"));
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value"));
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value"));
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value"));
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value"));
    sensors.push_back(std::string("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"));
    for (int i = 0; i < 100; i++) {
      values.clear();
        p->getSensors(sensors, values);
        for (int j = 0; j < values.size(); j++) {
            std::cout << sensors[j] << " = " << values[j] << std::endl;
        }
        std::cout << std::endl;
        SleepMs(1000);
    }


    exit ( 0 );
}

// kate: indent-mode cstyle; space-indent on; indent-width 0; 
