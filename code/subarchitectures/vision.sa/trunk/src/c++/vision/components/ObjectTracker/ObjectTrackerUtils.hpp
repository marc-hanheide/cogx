/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <VisionData.hpp>
#include <CDataFile.h>
#include "Tracker.h"
#include "TrackingEntry.h"

using namespace cast;
using namespace cogx;


struct Parameters{
	Tracking::Particle 	constraints;
	
	int									mode;
	int 								recursions;
	int 								particles;
	
	std::string 				modelPath;
	std::string 				texturePath;
	std::string 				shaderPath;
	
	float								edgeMatchingTol;
	bool								backFaceCulling;
	float								minTexGrabAngle;
};

// converts a VisionData::GeometryModel to a Tracker Model
bool convertGeometryModel(VisionData::GeometryModelPtr geom, Tracking::Model& model){
	unsigned int i;
	
	// Check if model structure is empty
	if(!geom){
		printf("[GeometryModel_Converter] no geometry found\n");
		return false;
	}

	// Parse through vertices and store content in Model
	Tracking::Vertex v;
	for(i=0; i<geom->vertices.size(); i++){
		v.pos.x = geom->vertices[i].pos.x;
		v.pos.y = geom->vertices[i].pos.y;
		v.pos.z = geom->vertices[i].pos.z;
		v.normal.x = geom->vertices[i].normal.x;
		v.normal.y = geom->vertices[i].normal.y;
		v.normal.z = geom->vertices[i].normal.z;
		v.texCoord.x = geom->vertices[i].texCoord.x;
		v.texCoord.y = geom->vertices[i].texCoord.y;
		model.push_back(v);
//	printf("Vertex: %f %f %f, %f %f %f \n", v.pos.x, v.pos.y, v.pos.z, v.normal.x, v.normal.y, v.normal.z);
	}
	
	// Parse through faces and store content in Model
	Tracking::Face f;
	for(i=0; i<geom->faces.size(); i++){	
		f.v = geom->faces[i].vertices;
		model.push_back(f);
		//printf("Face: %i %i %i %i\n", f.v[0], f.v[1], f.v[2], f.v[3]);
	}

	model.computeFaceNormals();
	
	/*
	for(i=0; i<(int)model->m_edgelist.size(); i++){
		printf(	"Edge: %i %i\n",
						model->m_edgelist[i].start,
						model->m_edgelist[i].end);
	}
	*/
	
	return true;
}

// converts a VisionData::GeometryModel to a Tracker Model
bool convertModel2Geometry(Tracking::Model& model, VisionData::GeometryModelPtr geom){
	int i;
	
	// Parse through vertices and store content in Model
	VisionData::Vertex vV;
	Tracking::Vertex tV;
	for(i=0; i<model.getVertexSize(); i++){
		tV = model.getVertex(i);
		vV.pos.x = tV.pos.x;
		vV.pos.y = tV.pos.y;
		vV.pos.z = tV.pos.z;
		vV.normal.x = tV.normal.x;
		vV.normal.y = tV.normal.y;
		vV.normal.z = tV.normal.z;
		vV.texCoord.x = tV.texCoord.x;
		vV.texCoord.y = tV.texCoord.y;
		geom->vertices.push_back(vV);
	}
	
	VisionData::Face vF;
	Tracking::Face tF;
	for(i=0; i<model.getFaceSize(); i++){
		tF = model.getFace(i);
		vF.vertices = tF.v;
		geom->faces.push_back(vF);
	}	

	// Parse through faces and store content in Model
// 	geom->faces = model.m_facelist;

	return true;
}

// converts a particle (x,y,z,alpha,beta,gamma) to a pose (R, t) 
bool convertParticle2Pose(Tracking::Pose& trPose, Math::Pose3& pose){
	mat3 rot;
	vec3 pos;
	
	trPose.getPose(rot, pos);
	
//	TODO Transposed (Michi?) (evtl. auch Fehler beim Laden der Matrix auf die GPU)
// 	pose.rot.m00 = rot[0]; pose.rot.m01 = rot[1]; pose.rot.m02 = rot[2];
// 	pose.rot.m10 = rot[3]; pose.rot.m11 = rot[4]; pose.rot.m12 = rot[5];
// 	pose.rot.m20 = rot[6]; pose.rot.m21 = rot[7]; pose.rot.m22 = rot[8];

	pose.rot.m00 = rot[0]; pose.rot.m01 = rot[3]; pose.rot.m02 = rot[6];
	pose.rot.m10 = rot[1]; pose.rot.m11 = rot[4]; pose.rot.m12 = rot[7];
	pose.rot.m20 = rot[2]; pose.rot.m21 = rot[5]; pose.rot.m22 = rot[8];
	
	pose.pos.x = pos.x;
	pose.pos.y = pos.y;
	pose.pos.z = pos.z;
	
	return true;
}

// converts a pose (R, t) to a particle (x,y,z,alpha,beta,gamma)
bool convertPose2Particle(Math::Pose3& pose, Tracking::Pose& trPose){
	mat3 rot;
	vec3 pos;
	
//	TODO Transposed (Michi?) (evtl. auch Fehler beim Laden der Matrix auf die GPU)
// 	rot[0] = (float)pose.rot.m00; rot[1] = (float)pose.rot.m01; rot[2] = (float)pose.rot.m02;
// 	rot[3] = (float)pose.rot.m10; rot[4] = (float)pose.rot.m11; rot[5] = (float)pose.rot.m12;
// 	rot[6] = (float)pose.rot.m20; rot[7] = (float)pose.rot.m21; rot[8] = (float)pose.rot.m22;
	

	rot[0] = (float)pose.rot.m00; rot[3] = (float)pose.rot.m01; rot[6] = (float)pose.rot.m02;
	rot[1] = (float)pose.rot.m10; rot[4] = (float)pose.rot.m11; rot[7] = (float)pose.rot.m12;
	rot[2] = (float)pose.rot.m20; rot[5] = (float)pose.rot.m21; rot[8] = (float)pose.rot.m22;

	pos.x = pose.pos.x;
	pos.y = pose.pos.y;
	pos.z = pose.pos.z;
	
	trPose.setPose(rot,pos);
	
	return true;
}

// converts time in seconds to CASTTime structure
cdl::CASTTime convertTime(double time_sec){
	cdl::CASTTime casttime;
	
	casttime.s = floor(time_sec);
	casttime.us = (time_sec - floor(time_sec)) * 1e6;
	
	return casttime;
}

double getFrameTime(cdl::CASTTime old_time, cdl::CASTTime new_time){
 	double dTime;
 	dTime = (new_time.s - old_time.s) + (new_time.us - old_time.us) * 1e-6;
	return dTime;
}

void convertCameraParameter(Video::CameraParameters vidCamPars, Tracking::CameraParameter& trackCamPars){
	
	trackCamPars.width	=	vidCamPars.width;
  trackCamPars.height	=	vidCamPars.height;
  
  trackCamPars.fx			=	vidCamPars.fx;
  trackCamPars.fy			=	vidCamPars.fy;
  trackCamPars.cx			=	vidCamPars.cx;
  trackCamPars.cy			=	vidCamPars.cy;
  
  trackCamPars.k1			=	vidCamPars.k1;
  trackCamPars.k2			=	vidCamPars.k2;
  trackCamPars.k3			=	vidCamPars.k3;
  
  trackCamPars.p1			=	vidCamPars.p1;
  trackCamPars.p2			=	vidCamPars.p2;
  
  trackCamPars.rot[0]	=	vidCamPars.pose.rot.m00;
  trackCamPars.rot[1]	=	vidCamPars.pose.rot.m01;
  trackCamPars.rot[2]	=	vidCamPars.pose.rot.m02;
  trackCamPars.rot[3]	=	vidCamPars.pose.rot.m10;
  trackCamPars.rot[4]	=	vidCamPars.pose.rot.m11;
  trackCamPars.rot[5]	=	vidCamPars.pose.rot.m12;
  trackCamPars.rot[6]	=	vidCamPars.pose.rot.m20;
  trackCamPars.rot[7]	=	vidCamPars.pose.rot.m21;
  trackCamPars.rot[8]	=	vidCamPars.pose.rot.m22;
  
  trackCamPars.pos.x	=	vidCamPars.pose.pos.x;
  trackCamPars.pos.y	=	vidCamPars.pose.pos.y;
  trackCamPars.pos.z	=	vidCamPars.pose.pos.z;
}


// SDL - Keyboard and Mouse input control
bool inputsControl(Tracking::Tracker* tracker, float fTimeTracker){
 	int i=0;
 	Tracking::Pose p;
 	
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
					printf("Kernel size: %d\n", (int)4);
					break;
				case SDLK_4:
					tracker->setKernelSize(2);
					printf("Kernel size: %d\n", (int)5);
					break;
				case SDLK_5:
					tracker->setKernelSize(2);
					printf("Kernel size: %d\n", (int)6);
					break;
				case SDLK_8:
					tracker->setEdgeShader();
					break;
				case SDLK_9:
					tracker->setColorShader();
					break;
				case SDLK_a:
					tracker->addModelFromFile("instantiations/ply-models/jasmin.ply", p, "jasmin");
					break;
				case SDLK_e:
					tracker->setEdgesImageFlag( !tracker->getEdgesImageFlag() );
					break;
				case SDLK_i:
					tracker->printStatistics();
					printf("\n	Total tracking time: %.0f ms\n", fTimeTracker*1000);
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
					tracker->saveModels("instantiations/ply-models/");
// 					tracker->saveScreenshot("screenshot.jpg");
					break;
				case SDLK_t:
					tracker->textureFromImage(false);
					break;
				case SDLK_u:
					tracker->untextureModels();
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






