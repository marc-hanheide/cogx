
#include <stdio.h>

#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"
#include "standalone.hpp"


#define EDGE 0
#define TEXTURE 1

Timer timer;

bool control(Tracker* tracker);

int main(int argc, char *argv[])
{
	char modelfilename[32];
	float likelihood = 0.0;
	int id = 0;
	IplImage* img = 0;
	TrackerModel* model = 0;
	Camera camera;
	Particle p_result = Particle(0.0);
	float fTimeIP, fTimeTrack, fTimeGrab;
	
	printf("\n\n");
	printf("---------------------\n");
	printf("- Object Tracker 3D -\n");
	printf("- by Thomas Mörwald -\n");
	printf("---------------------\n");
    
  // *************************************************************************************
  // Initialisation
	printf("\nInitialising\n");
	printf("---------------------\n");
	
	sprintf(modelfilename, "%s", "jasmin6.ply");
	
	Parameters params = LoadParametersFromINI("standalone.ini");
	
	p_result = params.initialParticle;

	// Setup resource manager
	g_Resources->SetModelPath(params.modelPath.c_str());
	g_Resources->SetTexturePath(params.texturePath.c_str());
	g_Resources->SetShaderPath(params.shaderPath.c_str());
	g_Resources->ShowLog(false);
	
	// Initialize camera capture using opencv
	g_Resources->InitCapture(params.camParams.width,params.camParams.height);
	img = g_Resources->GetNewImage();
	
	// Initialize SDL screen
	g_Resources->InitScreen(img->width, img->height, "Standalone Tracker");
	

#ifdef WIN32
	GLenum err = glewInit();
	if (GLEW_OK != err){
		// Problem: glewInit failed, something is seriously wrong.
		fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(err));
		return 1;
	}
#endif

	// Initialize tracker
	Tracker* m_tracker;
		
	if(params.mode == 0)
		m_tracker = new EdgeTracker();
	else if(params.mode == 1)
		m_tracker = new TextureTracker();
	else{
		printf("Wrong mode given in INI-file: %d\n", params.mode);
		return 1;
	}

	if(!m_tracker->init(	img->width, img->height,	// image size in pixels
												params.edgeMatchingTol,		// edge matching tolerance in degree
												params.minTexGrabAngle,
												p_result, 								// initial pose (where to reset when pressing 'z')
												params.constraints)){
		printf("Failed to initialise tracker!\n");
		return 1;
	}
	m_tracker->setBFC(params.backFaceCulling); // Disable Backface-Culling (required for non-volumetric objects like polyflaps)
	
	// Load extrensic camera parameters
	loadCameraParameters(camera, params.camParams, 0.1f, 10.0f);
		
	// Load model
	if((id = g_Resources->AddPlyModel(modelfilename)) == -1)
		return 1;
	model = g_Resources->GetModel(id);
	
	
	// *************************************************************************************
  // Main Loop
	printf("\nRunning (Mode = %i)\n", params.mode);
	printf("---------------------\n");
	while( control(m_tracker) ){
		// grab new image from camera
		timer.Update();
		img = g_Resources->GetNewImage();
		fTimeGrab = timer.Update();
		;
		// Image processing
		m_tracker->image_processing((unsigned char*)img->imageData);
		m_tracker->drawImage(NULL);
		fTimeIP = timer.Update();
		
		// Tracking (particle filtering)
		m_tracker->track(model, &camera, params.recursions, params.particles, params.constraints, p_result, 0.0);
		
		// Draw result
		m_tracker->drawResult(&p_result, model);
		m_tracker->drawCoordinates();
// 		m_tracker->drawSpeedBar(p_result.sp.x * 100.0);
// 		m_tracker->drawCalibrationPattern();
		m_tracker->swap();	
			
		fTimeTrack = timer.Update();
//  		prinf("grab: %.0f ip: %.0f track: %f\n",fTimeGrab*1000, fTimeIP*1000, fTimeTrack*1000);
	}
	
	// *************************************************************************************
  // Stop and destroy
	printf("\nStop\n");
	printf("---------------------\n");
	delete(g_Resources);
	delete(m_tracker);
	
	printf("\n... done\n\n");
	return 0;
}

