/*
 * DataSaver.cpp
 *
 *  Created on: Apr 5, 2011
 *      Author: alper
 */

#include "DataSaver.h"

using namespace cast;

extern "C" {
cast::CASTComponentPtr newComponent() {
	return new DataSaver();
}
}

DataSaver::DataSaver() {
}

void DataSaver::start() {
	startPCCServerCommunication(*this);
}

void DataSaver::configure(const std::map<std::string, std::string>& _config) {
	configureServerCommunication(_config);
}

DataSaver::~DataSaver() {
}


void DataSaver::receiveScan2d(const Laser::Scan2d &castScan){
	PointCloud::SurfacePointSeq points;
	getPoints(false,640, points);
}

void DataSaver::receiveOdometry(const Robotbase::Odometry &castOdom){

}
