/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#ifndef TRACKER_TOOLS_H_
#define TRACKER_TOOLS_H_

#include <stdio.h>
#include <math.h>
#include <string>
#include <stdexcept>

#include <Tracker/Tracker.h>
#include <Tracker/EdgeTracker.h>
#include <Tracker/TextureTracker.h>
#include <Tracker/Timer.h>
//#include "myPredictor.h"
#include <TomGine/tgMathlib.h>
#include <Tracker/CDataFile.h>
#include <GLWindow/GLWindow.h>

// using namespace Tracking;
// using namespace std;

namespace smlearning {

struct Parameters{
	int width;
	int height;
};


// *************************************************************************************
// Load INI File
string getModelFile(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return cdfParams.GetString("Model", "Files");
}

string getModelPath(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return cdfParams.GetString("ModelPath", "ResourcePath");
}

int getCamWidth(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return cdfParams.GetInt("w");
}

int getCamHeight(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return cdfParams.GetInt("h");
}


bool InputControl(Tracking::Tracker* tracker, blortGLWindow::Event& event){
	
	switch (event.type)
	{
	case blortGLWindow::TMGL_Press:
		switch (event.input)
		{
		case blortGLWindow::TMGL_q:
		case blortGLWindow::TMGL_Escape:
			return false;
			break;
		case blortGLWindow::TMGL_1: //1
			tracker->setKernelSize(0);
			printf("Kernel size: %d\n", (int)0);
			break;				
		case blortGLWindow::TMGL_2: //2
			tracker->setKernelSize(1);
			printf("Kernel size: %d\n", (int)1);
			break;				
		case blortGLWindow::TMGL_3: //3
			tracker->setKernelSize(2);
			printf("Kernel size: %d\n", (int)2);
			break;
		case blortGLWindow::TMGL_4: //4
			tracker->setEdgeShader();
			break;
		case blortGLWindow::TMGL_5: //5
			tracker->setColorShader();
			break;
		case blortGLWindow::TMGL_e: //e
			tracker->setEdgesImageFlag( !tracker->getEdgesImageFlag() );
			break;
		case blortGLWindow::TMGL_i: //i
			tracker->printStatistics();
			break;
		case blortGLWindow::TMGL_l: //l
			tracker->setLockFlag( !tracker->getLockFlag() );
			break;
		case blortGLWindow::TMGL_m: //m
			tracker->setModelModeFlag( tracker->getModelModeFlag()+1 );
			break;
		case blortGLWindow::TMGL_p: //p
			tracker->setDrawParticlesFlag( !tracker->getDrawParticlesFlag() );
			break;
		case blortGLWindow::TMGL_r: //r
			tracker->reset();
			break;
		case blortGLWindow::TMGL_s: //s
			tracker->saveModels("../Resources/ply/");
			break;				
		case blortGLWindow::TMGL_t: //t
			tracker->textureFromImage();
			break;
		case blortGLWindow::TMGL_u: //u
			tracker->untextureModels();
			break;
		default:
			break;
		}
		break;
	}
	return true;
}


} //namespace 

#endif 
