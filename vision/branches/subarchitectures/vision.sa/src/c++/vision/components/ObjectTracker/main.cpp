
#include <stdio.h>

#include "Tracker.h"
#include "Timer.h"

#define EDGE 0
#define TEXTURE 1
#define RENDERMODEL 2
#define RENDERIMAGE 3

int main(int argc, char *argv[])
{
	printf("\n\n");
	printf("---------------------\n");
	printf("- Object Tracker 3D -\n");
	printf("- by Thomas Mörwald -\n");
	printf("---------------------\n");
    
    printf("\nInitialising\n");
	printf("---------------------\n");
	
	int mode = TEXTURE;
	
	float apptime;
	float fTime;
	float likelihood = 0.0;
	int id = 0;
	IplImage* img = 0;
	Model* model = 0;
	Particle p_result = Particle(0.0);
	Timer timer;
	FILE* pFile = fopen("measurement.txt","w");
	
	
	
	
		
	// Initialize camera capture using opencv
	g_Resources->InitCapture(640,480);
	img = g_Resources->GetNewImage();
	
	// Initialize SDL scren
	g_Resources->InitScreen(img->width, img->height);
	
	// Initialize tracker
	Tracker m_tracker;
	
	if(mode == EDGE){
		// Set Parameter for edge tracking
		if(!m_tracker.init(	img->width, img->height,
							1000,
							49.0,
							0.25, 0.165, 0.25,
							0.0, 45.0,
							0.0, 0.1,
							17.0))
			return 0;
		
		// Load model with resource manager
		if((id = g_Resources->AddModel("box_blender.ply")) == -1)
			return 0;
		model = g_Resources->GetModel(id);
	}
	
	if(mode == TEXTURE){
		// Set Parameter for edge tracking
		if(!m_tracker.init(	img->width, img->height,
							2000,
							49.0,
							0.0, 0.18, 0.30,
							0.0, 40.0,
							0.0, 0.1,
							20.0,
							128, 128))
			return 0;
		
		// Load model with resource manager
		if((id = g_Resources->AddModel("casali.ply")) == -1)
			return 0;
		model = g_Resources->GetModel(id);
	}
	
	printf("\nRunning (Mode = %i)\n", mode);
	printf("---------------------\n");
	
	if(mode == EDGE){
		while(m_tracker.trackEdge((unsigned char*)img->imageData, model, &p_result, &p_result)){
			//timer.Update();
			//apptime = timer.GetApplicationTime();
			//fTime = timer.GetFrameTime();
			//fprintf(pFile, "%f %f %f %f %f %f %f\n", apptime, p_result.rX, p_result.rY, p_result.rZ, p_result.tX, p_result.tY, p_result.tZ);
			img = g_Resources->GetNewImage();
			//p_result.print();
		}
	}
	
	if(mode == TEXTURE){
		while(m_tracker.trackTexture((unsigned char*)img->imageData, model, &p_result, &p_result)){
			//timer.Update();
			//apptime = timer.GetApplicationTime();
			//fprintf(pFile, "%f %f %f %f %f %f %f\n", apptime, p_result.rX, p_result.rY, p_result.rZ, p_result.tX, p_result.tY, p_result.tZ);
			img = g_Resources->GetNewImage();
		}
	}
	
    
    printf("\nStop\n");
	printf("---------------------\n");
    m_tracker.release();
	delete(g_Resources);
	fclose(pFile);
	
	printf("\n... done\n\n");
	return 0;
}


