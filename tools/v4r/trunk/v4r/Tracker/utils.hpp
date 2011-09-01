/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <stdio.h>
#include <math.h>
#include <string>
#include <stdexcept>
#include <wordexp.h>


#include <v4r/Tracker/Tracker.h>
#include <v4r/Tracker/EdgeTracker.h>
#include <v4r/Tracker/TextureTracker.h>
#include <v4r/Tracker/Timer.h>

#include <v4r/Tracker/CDataFile.h>

#include <v4r/TomGine/tgTomGine.h>

using namespace Tracking;
using namespace std;

struct Parameters{
	int width;
	int height;
};

std::string expandName ( const std::string &fileString ) {
  std::stringstream ss;
    wordexp_t p;
    char** w;
    wordexp( fileString.c_str(), &p, 0 );
    w = p.we_wordv;
    for (size_t i=0; i < p.we_wordc; i++ ) {
      ss << w[i];
    }
    wordfree( &p );
    return ss.str();
}


// *************************************************************************************
// Load INI File
string getModelFile(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return expandName(cdfParams.GetString("Model", "Files"));
}

string getModelPath(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return expandName(cdfParams.GetString("ModelPath", "ResourcePath"));
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

void GetCameraParameter( TomGine::tgCamera::Parameter& camPar, const char* cam_cal_file, const char* pose_cal_file)
{
	CDataFile camCDF, poseCDF;

	// Load calibration and ini files
	if(!camCDF.Load(cam_cal_file)){
		char errmsg[128];
		sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open cam_cal file '%s'", cam_cal_file);
		throw std::runtime_error(errmsg);
	}

	if(!poseCDF.Load(pose_cal_file)){
		char errmsg[128];
		sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open pose_cal file '%s'", pose_cal_file);
		throw std::runtime_error(errmsg);
	}

	// Camera Parameters
	camPar.width = camCDF.GetInt("w");
	camPar.height = camCDF.GetInt("h");
	camPar.fx = camCDF.GetFloat("fx");
	camPar.fy = camCDF.GetFloat("fy");
	camPar.cx = camCDF.GetFloat("cx");
	camPar.cy = camCDF.GetFloat("cy");
	camPar.k1 = camCDF.GetFloat("k1");
	camPar.k2 = camCDF.GetFloat("k2");
	camPar.k3 = 0.0f;
	camPar.p1 = camCDF.GetFloat("p1");
	camPar.p2 = camCDF.GetFloat("p2");
	camPar.zFar = 5.0f;
	camPar.zNear = 0.1f;

	// Pose
	TomGine::vec3 p, r;
	std::string pose = poseCDF.GetString("pose");
	sscanf( pose.c_str(), "[%f %f %f] [%f %f %f]", &(p.x), &(p.y), &(p.z), &(r.x), &(r.y), &(r.z) );
 	//printf("%s\n", pose.c_str());
 	//printf("%f %f %f, %f %f %f\n", p.x, p.y, p.z, r.x, r.y, r.z);
	camPar.pos.x = p.x;
	camPar.pos.y = p.y;
	camPar.pos.z = p.z;
	camPar.rot.fromRotVector(r);
}

void GetTrackingParameter( Tracking::Tracker::Parameter& params, const char* ini_file)
{
	CDataFile cdfParams;

	if(!cdfParams.Load(ini_file)){
		char errmsg[128];
		sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open tracking_ini file '%s'", ini_file);
		throw std::runtime_error(errmsg);
	}

	// Tracking
	// Constraints
	params.variation.r.x = cdfParams.GetFloat("r.x", "Constraints") * M_PI/180.0f;
	params.variation.r.y = cdfParams.GetFloat("r.y", "Constraints") * M_PI/180.0f;
	params.variation.r.z = cdfParams.GetFloat("r.z", "Constraints") * M_PI/180.0f;
	params.variation.t.x 	= cdfParams.GetFloat("t.x", "Constraints");
	params.variation.t.y 	= cdfParams.GetFloat("t.y", "Constraints");
	params.variation.t.z 	= cdfParams.GetFloat("t.z", "Constraints");
	params.variation.z 		= cdfParams.GetFloat("z", "Constraints");

	// ParticleFilter
	params.num_recursions = cdfParams.GetInt("recursions", "ParticleFilter");
	params.num_particles = cdfParams.GetInt("particles", "ParticleFilter");
	params.convergence = cdfParams.GetInt("convergence", "ParticleFilter");
	params.pred_no_convergence = cdfParams.GetFloat("PredictorNoConvergence", "ParticleFilter");
	params.keep_best_particles = cdfParams.GetFloat("KeepBestParticles", "ParticleFilter");
	params.lpf = cdfParams.GetFloat("PoseLpfFactor", "ParticleFilter");

	// Resource Path
	params.modelPath = expandName(cdfParams.GetString("ModelPath", "ResourcePath"));
	params.texturePath = expandName(cdfParams.GetString("TexturePath", "ResourcePath"));
	params.shaderPath = expandName(cdfParams.GetString("ShaderPath", "ResourcePath"));

	// ImageProcessing
	params.edge_tolerance = cdfParams.GetFloat("EdgeMatchingTolerance", "ImageProcessing") * M_PI/180.0f;
	params.minTexGrabAngle = cdfParams.GetFloat("MinTextureGrabAngle", "ImageProcessing") * M_PI/180.0f;
	params.num_spreadings =  cdfParams.GetInt("NumberOfSpreadings", "ImageProcessing");
	params.max_kernel_size = cdfParams.GetInt("MaxKernelSize", "ImageProcessing");
	params.model_sobel_th = cdfParams.GetFloat("ModelSobelThreshold", "ImageProcessing");
	params.image_sobel_th = cdfParams.GetFloat("ImageSobelThreshold", "ImageProcessing");
	params.method = (Method)cdfParams.GetInt("Method", "ImageProcessing");
}


void printUsage(){
	printf("\n Tracker\n\n");

	printf(" Tracking control\n");
	printf(" -------------------------------------------\n");
	printf(" [q,Esc] Quit program\n");
	printf(" [1] Set kernel size to 1 pixel\n");
	printf(" [2] Set kernel size to 2 pixel\n");
	printf(" [3] Set kernel size to 3 pixel\n");
	printf(" [d] Enable/Disable Tracking-State-Detection\n");
	printf(" [e] Show edges of image\n");
	printf(" [i] Print tracking information to console\n");
	printf(" [l] Lock/Unlock tracking\n");
	printf(" [m] Switch display mode of model\n");
	printf(" [p] Show particles\n");
	printf(" [r] Reset tracker to initial pose\n");
	printf(" [s] Save model to file (including textures)\n");
	printf(" [t] Capture model texture from image\n");
	printf(" [u] Untexture/remove texture from model\n");
	printf(" \n\n");
}

bool InputControl(Tracker* tracker, V4R::Event& event){
	
	switch (event.type)
	{
	case V4R::TMGL_Press:
		switch (event.input)
		{
		case V4R::TMGL_q:
		case V4R::TMGL_Escape:
			return false;
			break;
		case V4R::TMGL_1: //1
			tracker->setKernelSize(0);
			printf("Kernel size: %d\n", (int)0);
			break;				
		case V4R::TMGL_2: //2
			tracker->setKernelSize(1);
			printf("Kernel size: %d\n", (int)1);
			break;				
		case V4R::TMGL_3: //3
			tracker->setKernelSize(2);
			printf("Kernel size: %d\n", (int)2);
			break;
		case V4R::TMGL_d:
			tracker->setTrackingStateDetection( !tracker->getTrackingStateDetection() );
			break;
		case V4R::TMGL_e: //e
			tracker->setEdgesImageFlag( !tracker->getEdgesImageFlag() );
			break;
		case V4R::TMGL_i: //i
			tracker->printParams();
			tracker->printStatistics();
			break;
		case V4R::TMGL_l: //l
			tracker->setLockFlag( !tracker->getLockFlag() );
			break;
		case V4R::TMGL_m: //m
			tracker->setModelModeFlag( tracker->getModelModeFlag()+1 );
			break;
		case V4R::TMGL_p: //p
			tracker->setDrawParticlesFlag( !tracker->getDrawParticlesFlag() );
			break;
		case V4R::TMGL_r: //r
			tracker->reset();
			break;
		case V4R::TMGL_s: //s
			tracker->saveModels("./");
			break;				
		case V4R::TMGL_t: //t
			tracker->textureFromImage();
			break;
		case V4R::TMGL_u: //u
			tracker->untextureModels();
			break;
		default:
			break;
		}
		break;
	}
	return true;
}




