
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
	//float apptime;
	//float t1, t2;
	char modelfilename[32];
	float likelihood = 0.0;
	int id = 0;
	IplImage* img = 0;
	Model* model = 0;
	Camera* camera = 0;
	Particle p_result = Particle(0.0);
	Particle p_constraints = Particle(1.0);
	
	float fTimeIP, fTimeTrack, fTimeGrab;
	
	printf("\n\n");
	printf("---------------------\n");
	printf("- Object Tracker 3D -\n");
	printf("- by Thomas Mörwald -\n");
	printf("---------------------\n");
    
	printf("\nInitialising\n");
	printf("---------------------\n");
	
	// Type in your parameters from cam.cal and pose.cal
	// The values now are with respect to the two files in this
	// directory 'example-cam.cal' and 'example-pose.cal'
	// BEGIN EDIT
	CameraParameters camPar;
	camPar.width = 640;
	camPar.height = 480;
	camPar.fx = 527.727234;
	camPar.fy = 522.151367;
	camPar.cx = 306.918823;
	camPar.cy = 239.443680;
	camPar.k1 = 0.072731;
	camPar.k2 = -0.228507;
	camPar.k3 = 0.0;
	camPar.p1 = -0.002634;
	camPar.p2 = -0.000522;
	camPar.pose.pos.x = 0.155897;
	camPar.pose.pos.y = -0.224653;
	camPar.pose.pos.z = 0.281556;
	vec3 vRot;
	vRot.x = -2.134039;
	vRot.y = 0.059238;
	vRot.z = -0.048697;
	// END EDIT

	// conversion from pose.cal rotation vector to rotation matrix
	mat3 vMat;
	fromRotVector(vMat, vRot);
	camPar.pose.rot = vMat;
  
  int mode = TEXTURE;
  
	sprintf(modelfilename, "%s", "cylinder.ply");
	
	// TEXTURE
	// Robust and accurate, low framerate
	int num_recursions = 4;
	int num_particles = 200;				//TODO Dynamic (motion and attention based) number of particles and recursions
	float rot = 15.0 * PIOVER180;
	float rotp = 2.0;
	float trans = 0.01;
	float transp = 0.2;
	float zoom = 0.01;
	float zoomp = 0.5;
	
// 	// High framerate, not robust and not accurate
// 	int num_recursions = 2;
// 	int num_particles = 50;
// 	float rot = 10.0 * PIOVER180;
// 	float rotp = 2.0;
// 	float trans = 0.005;
// 	float transp = 0.1;
// 	float zoom = 0.01;
// 	float zoomp = 1.0;

	p_result.translate(0.2,0.1,0.06);

	p_constraints.r.x = rot;
	p_constraints.r.y = rot;
	p_constraints.r.z = rot;
	p_constraints.rp.x = rotp;
	p_constraints.rp.y = rotp;
	p_constraints.rp.z = rotp;
	p_constraints.s.x = trans;
	p_constraints.s.y = trans;
	p_constraints.s.z = trans;
	p_constraints.sp.x = transp;
	p_constraints.sp.y = transp;
	p_constraints.sp.z = transp;
	p_constraints.z = zoom;	
	p_constraints.zp = zoomp;
	
	// Set pathes of resource manager
	g_Resources->SetModelPath("resources/model/");
	g_Resources->SetTexturePath("resources/texture/");
	g_Resources->SetShaderPath("resources/shader/");
			
	// Initialize camera capture using opencv
	g_Resources->InitCapture(camPar.width,camPar.height);
	img = g_Resources->GetNewImage();
	
	// Initialize SDL screen
	g_Resources->InitScreen(img->width, img->height);
	
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
		
	if(mode == EDGE){
		g_Resources->ShowLog(true);
		m_tracker = new EdgeTracker();
		if(!m_tracker->init(	img->width, img->height,	// image size in pixels
								num_particles,																// number of particles for each recursion (lower if tracker consumes too much power)
								num_recursions,													// recursions of particle filter (lower if tracker consumes too much power)
								float(45.0*PIOVER180),							// edge matching tolerance in degree
								0.05f,															// goal tracking time in seconds (not implemented yet)
								p_result))													// initial pose (where to reset when pressing 'z')
			return 0;
	}
	
	if(mode == TEXTURE){
		g_Resources->ShowLog(true);
		m_tracker = new TextureTracker();
		if(!m_tracker->init(	img->width, img->height,	// image size in pixels
								num_particles,																// maximum number of particles
								num_recursions,													// recursions of particle filter (lower if tracker consumes too much power)
								float(45.0*PIOVER180),							// edge matching tolerance in degree
								0.02f,															// goal tracking time in seconds
								p_result))													// initial pose (where to reset when pressing 'z')
			return 0;
	}
	
	// Load extrensic Camera
	if((id = g_Resources->AddCamera("cam_extrinsic")) == -1)
		return 0;
	camera = g_Resources->GetCamera(id);
	camera->Set(	0.25, 0.25, 0.25,											// Position of camera relative to Object
								0.0, 0.0, 0.0,											// Point where camera looks at (world origin)
								0.0, 1.0, 0.0,											// Up vector (y-axis)
								45, img->width, img->height,		  // field of view angle, image width and height
								0.1, 10.0,													// camera z-clipping planes (far, near)
								GL_PERSPECTIVE);										// Type of projection (GL_ORTHO, GL_PERSPECTIVE)
	loadCameraParameters(camera, camPar, 0.1f, 10.0f);
		
	// Load model with resource manager
//  	if((id = g_Resources->AddPlyModel("box_red.ply")) == -1)
	if((id = g_Resources->AddPlyModel(modelfilename)) == -1)
		return 0;
	model = g_Resources->GetModel(id);
	
	// Disable Backface-Culling (required for non-volumetric objects like polyflaps)
	m_tracker->setBFC(true);
	
	printf("\nRunning (Mode = %i)\n", mode);
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
		m_tracker->track(model, camera, num_recursions, num_particles, p_constraints, p_result, 0.0);
		
		// Draw result
// 		m_tracker->drawImage(NULL);
		m_tracker->drawResult(&p_result, model);
// 		m_tracker->drawCoordinates();
// 		m_tracker->drawSpeedBar(p_result.sp.x * 100.0);
// 		m_tracker->drawTest();
		m_tracker->swap();	
			
		fTimeTrack = timer.Update();
//  		prinf("grab: %.0f ip: %.0f track: %f\n",fTimeGrab*1000, fTimeIP*1000, fTimeTrack*1000);
	}
	
// 	int iM = 20;
// 	int nr = 3;
// 	for(int np=10; np<200; np+=10){
// 		fTimeGrab = fTimeIP = fTimeTrack = 0.0;
// 		for(int i=0; i<iM; i++){
// 			// grab new image from camera
// 			timer.Update();
// 			img = g_Resources->GetNewImage();
// 			fTimeGrab += timer.Update();
// 			
// 			// Image processing
// 			m_tracker->image_processing((unsigned char*)img->imageData);
// 			fTimeIP += timer.Update();
// 			
// 			// Tracking (particle filtering)
// 			m_tracker->track(model, camera, nr, np, p_constraints, p_result, 0.0);
// 			m_tracker->swap();
// 			fTimeTrack += timer.Update();
// 			
// 			// Draw result
// 			m_tracker->drawImage(NULL);
// 			m_tracker->drawResult(&p_result, model);
// 			m_tracker->drawCoordinates();
// 			m_tracker->swap();
// 		}
			
// 		printf("%d %f %f %f\n",np, fTimeGrab*1000/iM, fTimeIP*1000/iM, fTimeTrack*1000/iM);
// 	}

	

	printf("\nStop\n");
	printf("---------------------\n");
	delete(g_Resources);
	delete(m_tracker);
	
	printf("\n... done\n\n");
	return 0;
}

bool control(Tracker* tracker){
 	char filename[16];
 	vector<float> pdfmap;
 	Particle pose;
 	float s;
 	int res;
 	
	SDL_Event event;
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
				case SDLK_e:
					tracker->showEdgesImage( !tracker->getEdgesImage() );
					break;
				case SDLK_l:
					tracker->lock( !tracker->getLock() );
					break;
				case SDLK_m:
					tracker->showEdgesModel( !tracker->getEdgesModel() );
					break;
				case SDLK_p:
					tracker->showParticles( !tracker->getParticlesVisible() );
					break;
				case SDLK_s:
					tracker->showStatistics();
					break;
				case SDLK_t:
					tracker->textureFromImage();
					break;
				case SDLK_u:
					tracker->setTestflag(true);
					break;	
				case SDLK_w:
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
					
					break;
				case SDLK_z:
					tracker->zeroParticles();
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


