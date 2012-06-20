
#include <stdio.h>

#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"
#include "myPredictor.h"
#include "standalone.hpp"

int main(int argc, char *argv[])
{
	float likelihood = 0.0;
	Timer timer;
	IplImage* img = 0;
	ModelLoader m_modelloader;
	float fTimeIP, fTimeTrack, fTimeGrab, fTimeTotal;
	Pose p_result;
	SDL_Event event;
	int i;
	
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
// 	Model model_1, model_2;
// 	TrackerModel model_3;
// 	m_modelloader.LoadPly(model_1, "resources/model/red_box.ply");
// 	p.t = vec3(0.2, 0.06, 0.06);
// 	id_1 = m_tracker->addModel(model_1, p, true);

	std::vector<vec3> m_points;
		
// 	m_modelloader.LoadPly(model_2, "resources/model/GuteLaune.ply");
// 	m_modelloader.LoadPly(&model_3, "resources/model/Object-1.ply");
	
// 	model_2.print();
	p.t = vec3(0.05, 0.1, 0.05);
// 	id_2 = m_tracker->addModel(model_2, p, "Object-1", true);
	id_2 = m_tracker->addModelFromFile("resources/model/jasmin6.ply", p, "jasmin6", true);

	// *************************************************************************************
  // Main Loop
	printf("\nRunning \n");
	printf("---------------------\n");
	while( control(m_tracker, event) ){
		// grab new image from camera
		fTimeTotal = 0.0;
		timer.Update();
		img = g_Resources->GetNewImage();
		fTimeGrab = timer.Update();
		
		// Image processing
		m_tracker->image_processing((unsigned char*)img->imageData);
		fTimeIP = timer.Update();
		
		// Tracking (particle filtering)
		m_tracker->track();
		
		// Draw result
		m_tracker->drawImage(0);
		m_tracker->drawResult();
		m_tracker->drawCoordinates();
		
		// Draw Points on Model
		m_tracker->getModelPose(id_2, p_result);
		glDisable(GL_DEPTH_TEST);
		p_result.activate();
			glColor3f(1.0,0.0,0.0);
			glPointSize(2);
			glBegin(GL_LINE_STRIP);
			for(i=0; i<m_points.size(); i++){
				glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
			}
			glEnd();
			glColor3f(1.0,1.0,1.0);
		p_result.deactivate();
		
		
		m_tracker->swap();
		
		// Get Point on Model
		if(event.type == SDL_MOUSEBUTTONDOWN){
			vec3 v;
			if(m_tracker->getModelPoint3D(id_2, event.button.x, event.button.y, v.x, v.y, v.z)){
				m_points.push_back(v);
			}
		}

		fTimeTrack = timer.Update();
		fTimeTotal = fTimeGrab + fTimeIP + fTimeTrack;
// 		m_tracker->setFrameTime(fTimeTotal);
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

