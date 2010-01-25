
#include <stdio.h>

#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"
#include "standalone.hpp"

int main(int argc, char *argv[])
{
	float likelihood = 0.0;
	Timer timer;
	IplImage* img = 0;
	ModelLoader m_modelloader;
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
	
	string ini_file = string("standalone.ini");	
	
	// Initialize camera capture using opencv
	int width = 640;
	int height = 480;
	g_Resources->InitCapture(width, height);
	img = g_Resources->GetNewImage();

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
	int mode = 1;
	
	if(mode == 0)
		m_tracker = new EdgeTracker();
	else if(mode == 1)
		m_tracker = new TextureTracker();
	else{
		printf("Wrong mode selected (only 0 or 1 possible)\n", mode);
		return 1;
	}

	if(!m_tracker->init(ini_file.c_str(), img->width, img->height)){
		printf("Failed to initialise tracker!\n");
		return 1;
	}
	
	// Load model
	int id_1, id_2;
	Pose p;
	Model model_1, model_2;
	
	m_modelloader.LoadPly(model_1, "resources/model/red_box.ply");
	p.t = vec3(0.2, 0.06, 0.06);
	id_1 = m_tracker->addModel(model_1, p, true);
	
	m_modelloader.LoadPly(model_2, "resources/model/jasmin6.ply");
	p.t = vec3(0.05, 0.05, 0.05);
	id_2 = m_tracker->addModel(model_2, p, true);
	
	// *************************************************************************************
  // Main Loop
	printf("\nRunning \n");
	printf("---------------------\n");
	while( control(m_tracker) ){
		// grab new image from camera
		timer.Update();
		img = g_Resources->GetNewImage();
		fTimeGrab = timer.Update();
		
		// Image processing
		m_tracker->image_processing((unsigned char*)img->imageData);
		m_tracker->drawImage(NULL);
		fTimeIP = timer.Update();
		
		// Tracking (particle filtering)
		m_tracker->track();
		
		// Draw result
		m_tracker->drawResult();
// 		m_tracker->drawCoordinates();
		m_tracker->swap();	
			
		fTimeTrack = timer.Update();
//  		prinf("grab: %.0f ip: %.0f track: %f\n",fTimeGrab*1000, fTimeIP*1000, fTimeTrack*1000);
	}
	
	// *************************************************************************************
  // Stop and destroy
	printf("\nStop\n");
	printf("---------------------\n");

	delete(m_tracker);
	
	printf("\n... done\n\n");
	return 0;
}

