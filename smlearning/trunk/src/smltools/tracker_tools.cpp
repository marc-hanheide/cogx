/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */
#include <smltools/tracker_tools.h>

namespace smlearning {

// *************************************************************************************
// Load INI File
std::string getModelFile(const char* filename){
	CDataFile cdfParams;

	if(!cdfParams.Load(filename)){
		char errmsg[128];
		snprintf(errmsg, 128, "[GetTrackingParameter] Can not open file '%s'", filename);
		throw std::runtime_error(errmsg);
	}
	
	return cdfParams.GetString("Model", "Files");
}

std::string getModelPath(const char* filename){
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
	case blortGLWindow::TMGL_Release:
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

void GetTrackingParameter( Tracking::Tracker::Parameter& params, const char* ini_file)
{
	CDataFile iniCDF;
	
	if(!iniCDF.Load(ini_file)){
		char errmsg[128];
		sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open tracking_ini file '%s'", ini_file);
		throw std::runtime_error(errmsg);
	}
	
	// Tracking
	// Constraints
	params.variation.r.x = iniCDF.GetFloat("r.x", "Constraints") * Tracking::pi/180.0f;
	params.variation.r.y = iniCDF.GetFloat("r.y", "Constraints") * Tracking::pi/180.0f;
	params.variation.r.z = iniCDF.GetFloat("r.z", "Constraints") * Tracking::pi/180.0f;
	params.variation.t.x 	= iniCDF.GetFloat("t.x", "Constraints");
	params.variation.t.y 	= iniCDF.GetFloat("t.y", "Constraints");
	params.variation.t.z 	= iniCDF.GetFloat("t.z", "Constraints");
	params.variation.z 		= iniCDF.GetFloat("z", "Constraints");
	
	// Performance
	params.num_recursions = iniCDF.GetInt("recursions", "Performance");
	params.num_particles = iniCDF.GetInt("particles", "Performance");
	params.hypotheses_trials = iniCDF.GetInt("hypotheses", "Performance");
	params.convergence = iniCDF.GetInt("convergence", "Performance");
	
	// Resource Path
	params.modelPath = iniCDF.GetString("ModelPath", "ResourcePath");
	params.texturePath = iniCDF.GetString("TexturePath", "ResourcePath");
	params.shaderPath = iniCDF.GetString("ShaderPath", "ResourcePath");
		
	// Other
	params.edge_tolerance = iniCDF.GetFloat("EdgeMatchingTolerance", "Other") * Tracking::pi/180.0f;
	params.minTexGrabAngle = iniCDF.GetFloat("MinTextureGrabAngle", "Other") * Tracking::pi/180.0f;
	params.num_spreadings =  iniCDF.GetInt("NumberOfSpreadings", "Other");
	params.max_kernel_size = iniCDF.GetInt("MaxKernelSize", "Other");
	
	params.model_sobel_th = iniCDF.GetFloat("ModelSobelThreshold", "Other");
	params.image_sobel_th = iniCDF.GetFloat("ImageSobelThreshold", "Other");
	params.pred_no_convergence = iniCDF.GetFloat("PredictorNoConvergence", "Other");
	
	params.c_th_base = iniCDF.GetFloat("BaseThreshold", "Qualitative");
	params.c_th_min = iniCDF.GetFloat("MinThreshold", "Qualitative");
	params.c_th_fair = iniCDF.GetFloat("FairThreshold", "Qualitative");
	params.c_th_lost = iniCDF.GetFloat("LostThreshold", "Qualitative");
	
	params.c_mv_not = iniCDF.GetFloat("NoMovementThreshold", "Movement");
	params.c_mv_slow = iniCDF.GetFloat("SlowMovementThreshold", "Movement");
}


} // namespace
