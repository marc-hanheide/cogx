/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include "Tracker.h"
#include "mathlib.h"
#include <math.h>
#include <string>
#include "CDataFile.h"

using namespace Tracking;
using namespace std;

struct Parameters{
	int width;
	int height;
};


// *************************************************************************************
// Load INI File
Parameters LoadParametersFromINI(const char* filename){
	
	Parameters params;
	
	CDataFile cdfParams;
	cdfParams.Load(filename);
	
	// Camera Parameters
	params.width = cdfParams.GetInt("width", "CameraParameters");
	params.height = cdfParams.GetInt("height", "CameraParameters");
		
	return params;
}

// *************************************************************************************
// Input Controls
bool control(Tracker* tracker, SDL_Event &event){
 	char filename[16];
 	vector<float> pdfmap;
 	Particle pose;
 	float s;
 	int res;
 	
	while(SDL_PollEvent(&event)){
		switch(event.type){
		case SDL_KEYDOWN:
			switch(event.key.keysym.sym){
				case SDLK_ESCAPE:
					return false;
					break;
				case SDLK_1:
					tracker->setKernelSize(0);
					printf("Kernel size: %d\n", (int)0);
					break;				
				case SDLK_2:
					tracker->setKernelSize(1);
					printf("Kernel size: %d\n", (int)1);
					break;				
				case SDLK_3:
					tracker->setKernelSize(2);
					printf("Kernel size: %d\n", (int)2);
					break;
				case SDLK_4:
					tracker->setEdgeShader();
					break;
				case SDLK_5:
					tracker->setColorShader();
					break;
				case SDLK_a:
					tracker->evaluatePDF(0, -0.05, -0.05, 0.05, 0.05, 256, "jasmin1.pdf", "jasmin1.xpdf");
					break;
				case SDLK_e:
					tracker->setEdgesImageFlag( !tracker->getEdgesImageFlag() );
					break;
				case SDLK_i:
					tracker->printStatistics();
					break;
				case SDLK_l:
					tracker->setLockFlag( !tracker->getLockFlag() );
					break;
				case SDLK_m:
					tracker->setModelModeFlag( tracker->getModelModeFlag()+1 );
					break;
				case SDLK_p:
					tracker->setDrawParticlesFlag( !tracker->getDrawParticlesFlag() );
					break;
				case SDLK_s:
					tracker->saveModels("resources/model/");
					break;				
				case SDLK_t:
					tracker->textureFromImage();
					break;
				case SDLK_u:
					tracker->untextureModels();
					break;
				case SDLK_w:
					/*
					pose = tracker->getLastPose();
					tracker->setSpreadLvl(4);
					s = 0.1;
					res = 256;
					
					tracker->setKernelSize(1);
					pdfmap = tracker->getPDFxy(pose,-s,-s,s,s,res);
					tracker->savePDF(pdfmap,-s,-s,s,s,res,"graphs/kernel_1.ply", "graphs/kernel_1.dat");
					
					tracker->setKernelSize(2);
					pdfmap = tracker->getPDFxy(pose,-s,-s,s,s,res);
					tracker->savePDF(pdfmap,-s,-s,s,s,res,"graphs/kernel_2.ply", "graphs/kernel_2.dat");
					
					tracker->setKernelSize(3);
					pdfmap = tracker->getPDFxy(pose,-s,-s,s,s,res);
					tracker->savePDF(pdfmap,-s,-s,s,s,res,"graphs/kernel_3.ply", "graphs/kernel_3.dat");
					*/
					break;
				case SDLK_z:
					tracker->reset();
					break;
                default:
					break;
			}
			break;
		case SDL_QUIT:
			return false;
			break;
		default:
			break;
		}
	}
	return true;
}





