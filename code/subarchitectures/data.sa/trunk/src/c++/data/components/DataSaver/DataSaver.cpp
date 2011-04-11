/*
 * DataSaver.cpp
 *
 *  Created on: Apr 5, 2011
 *      Author: alper
 */

#include "DataSaver.h"


extern "C" {
cast::CASTComponentPtr newComponent() {
	return new AVS_ContinualPlanner();
}
}

DataSaver::DataSaver() {
	// TODO Auto-generated constructor stub

}

DataSaver::~DataSaver() {
	// TODO Auto-generated destructor stub
}
