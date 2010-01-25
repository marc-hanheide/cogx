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
using namespace Math;


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

/* // converts a Tracker Model to a VisionData::GeometryModel
// bool convertTrackerModel(Tracking::TrackerModel* model, VisionData::GeometryModelPtr geom){
// 	unsigned int i;
// 	
// 	if(!model){
// 		printf("[TrackerModel_Converter] no geometry found\n");
// 		return false;
// 	}
// 	
// 	// Parse through vertices and store content in Model
// 	for(i=0; i<model->m_vertexlist.size(); i++){
// 		VisionData::Vertex v;
// 		v.pos.x = model->m_vertexlist[i].pos.x;
// 		v.pos.y = model->m_vertexlist[i].pos.y;
// 		v.pos.z = model->m_vertexlist[i].pos.z;
// 		v.normal.x = model->m_vertexlist[i].normal.x;
// 		v.normal.y = model->m_vertexlist[i].normal.y;
// 		v.normal.z = model->m_vertexlist[i].normal.z;
// 		v.texCoord.x = model->m_vertexlist[i].texCoord.x;
// 		v.texCoord.y = model->m_vertexlist[i].texCoord.y;
// 		geom->vertices.push_back(v);
// 		//printf("Vertex: %f %f %f, %f %f %f \n", v.pos.x, v.pos.y, v.pos.z, v.normal.x, v.normal.y, v.normal.z);
// 	}
// 	
// 	// Parse through faces and store content in Model
// 	for(i=0; i<model->m_facelist.size(); i++){
// 		VisionData::Face f;
// 		f.vertices = model->m_facelist[i].v;
// 		geom->faces.push_back(f);	
// 	}
// 	
// 	
// 	return true;
// }
*/

// converts a particle (x,y,z,alpha,beta,gamma) to a pose (R, t) 
bool convertParticle2Pose(Tracking::Pose& trPose, Pose3& pose){
	mat3 rot;
	vec3 pos;
	
	trPose.getPose(rot, pos);
	
	pose.rot.m00 = rot[0]; pose.rot.m01 = rot[1]; pose.rot.m02 = rot[2];
	pose.rot.m10 = rot[3]; pose.rot.m11 = rot[4]; pose.rot.m12 = rot[5];
	pose.rot.m20 = rot[6]; pose.rot.m21 = rot[7]; pose.rot.m22 = rot[8];
	
	pose.pos.x = pos[0];
	pose.pos.y = pos[1];
	pose.pos.z = pos[2];
	
	return true;
}

// converts a pose (R, t) to a particle (x,y,z,alpha,beta,gamma)
bool convertPose2Particle(Pose3& pose, Tracking::Pose& trPose){
	mat3 rot;
	vec3 pos;
	
	rot[0] = (float)pose.rot.m00; rot[1] = (float)pose.rot.m01; rot[2] = (float)pose.rot.m02;
	rot[3] = (float)pose.rot.m10; rot[4] = (float)pose.rot.m11; rot[5] = (float)pose.rot.m12;
	rot[6] = (float)pose.rot.m20; rot[7] = (float)pose.rot.m21; rot[8] = (float)pose.rot.m22;
	
	pos.x = pose.pos.x;
	pos.y = pose.pos.y;
	pos.z = pose.pos.z;

	trPose.setPose(rot, pos);
	
	return true;
}

// converts time in seconds to CASTTime structure
cdl::CASTTime convertTime(double time_sec){
	cdl::CASTTime casttime;
	
	casttime.s = floor(time_sec);
	casttime.us = (time_sec - floor(time_sec)) * 1e6;
	
	return casttime;
}

// Converts Video::CameraParameters from Video::Image of VideoServer to 
// Extrinsic- and Intrinsic- Matrix of OpenGL
// zNear and zFar describe the near and far z values of the clipping plane
void loadCameraParameters(Tracking::Camera* camera, Video::CameraParameters camPars, float zNear, float zFar){
	// intrinsic parameters
	// transform the coordinate system of computer vision to OpenGL 
	//   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
	//   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
	float fx = 2.0*camPars.fx / camPars.width;					// scale range from [0 ... 640] to [0 ... 2]
  float fy = 2.0*camPars.fy / camPars.height;					// scale range from [0 ... 480] to [0 ... 2]
  float cx = 1.0-(2.0*camPars.cx / camPars.width);		// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float cy = (2.0*camPars.cy / camPars.height)-1.0;		// flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float z1 = (zFar+zNear)/(zNear-zFar);								// entries for clipping planes
  float z2 = 2*zFar*zNear/(zNear-zFar);								// look up for gluPerspective
  
  // intrinsic matrix
  mat4 intrinsic;
  intrinsic[0]=fx;	intrinsic[4]=0;		intrinsic[8]=cx;	intrinsic[12]=0;
  intrinsic[1]=0;		intrinsic[5]=fy;	intrinsic[9]=cy;	intrinsic[13]=0;
  intrinsic[2]=0;		intrinsic[6]=0;		intrinsic[10]=z1;	intrinsic[14]=z2;  
  intrinsic[3]=0;		intrinsic[7]=0;		intrinsic[11]=-1;	intrinsic[15]=0;	// last row assigns w=-z which inverts cx and cy at w-division
  
  // computer vision camera coordinates to OpenGL camera coordinates transform 
  // rotate 180° about x-axis
  mat4 cv2gl;
  cv2gl[0]=1.0; cv2gl[4]=0.0; 	cv2gl[8]=0.0;   cv2gl[12]=0.0;  
	cv2gl[1]=0.0; cv2gl[5]=-1.0;	cv2gl[9]=0.0;   cv2gl[13]=0.0;  
	cv2gl[2]=0.0; cv2gl[6]=0.0; 	cv2gl[10]=-1.0; cv2gl[14]=0.0;  
	cv2gl[3]=0.0; cv2gl[7]=0.0; 	cv2gl[11]=0.0;  cv2gl[15]=1.0;  
	
	// extrinsic parameters
	// look up comments in tools/hardware/video/src/slice/Video.ice
	// p = R^T*(w - t) = (R^T, -R^T*t) * (w,1)
	cogx::Math::Matrix33 R = camPars.pose.rot;
	cogx::Math::Vector3 t = camPars.pose.pos;
	mat4 extrinsic;
	extrinsic[0]=R.m00;	extrinsic[4]=R.m01;	extrinsic[8]=R.m02;		extrinsic[12]=0.0;
	extrinsic[1]=R.m10;	extrinsic[5]=R.m11;	extrinsic[9]=R.m12;		extrinsic[13]=0.0;	
	extrinsic[2]=R.m20;	extrinsic[6]=R.m21;	extrinsic[10]=R.m22;	extrinsic[14]=0.0;	
	extrinsic[3]=0.0;		extrinsic[7]=0.0;		extrinsic[11]=0.0;		extrinsic[15]=1.0;
	extrinsic = extrinsic.transpose();											// R^T
	vec4 tp = -(extrinsic * vec4(t.x, t.y, t.z, 1.0));			// -R^T*t
	extrinsic[12]=tp.x; extrinsic[13]=tp.y; extrinsic[14]=tp.z;
	extrinsic = cv2gl * extrinsic;
	
	// set camera parameters
	camera->SetViewport(camPars.width,camPars.height);
	camera->SetZRange(zNear, zFar);
	camera->SetIntrinsic(intrinsic);
	camera->SetExtrinsic(extrinsic);  
	camera->SetPos(camPars.pose.pos.x, camPars.pose.pos.y, camPars.pose.pos.z);
}

// *************************************************************************************
// Load INI File
Parameters LoadParametersFromINI(const char* filename){
	
	Parameters params;
	
	CDataFile cdfParams;
	cdfParams.Load(filename);
	
	// Constraints
// 	params.constraints.r.x 	= cdfParams.GetFloat("r.x", "Constraints") * PIOVER180;
// 	params.constraints.r.y 	= cdfParams.GetFloat("r.y", "Constraints") * PIOVER180;
// 	params.constraints.r.z 	= cdfParams.GetFloat("r.z", "Constraints") * PIOVER180;
	params.constraints.rp.x = cdfParams.GetFloat("rp.x", "Constraints");
	params.constraints.rp.y = cdfParams.GetFloat("rp.y", "Constraints");
	params.constraints.rp.z = cdfParams.GetFloat("rp.z", "Constraints");
	params.constraints.t.x 	= cdfParams.GetFloat("s.x", "Constraints");
	params.constraints.t.y 	= cdfParams.GetFloat("s.y", "Constraints");
	params.constraints.t.z 	= cdfParams.GetFloat("s.z", "Constraints");
	params.constraints.tp.x = cdfParams.GetFloat("sp.x", "Constraints");
	params.constraints.tp.y = cdfParams.GetFloat("sp.y", "Constraints");
	params.constraints.tp.z = cdfParams.GetFloat("sp.z", "Constraints");
	params.constraints.z 		= cdfParams.GetFloat("z", "Constraints");	
	params.constraints.zp 	= cdfParams.GetFloat("zp", "Constraints");
	
	// Performance
	params.mode = cdfParams.GetInt("mode", "Performance");
	params.recursions = cdfParams.GetInt("recursions", "Performance");
	params.particles = cdfParams.GetInt("particles", "Performance");
	
	// Resource Path
	params.modelPath = cdfParams.GetString("ModelPath", "ResourcePath");
	params.texturePath = cdfParams.GetString("TexturePath", "ResourcePath");
	params.shaderPath = cdfParams.GetString("ShaderPath", "ResourcePath");
		
	// Other
	params.edgeMatchingTol = cdfParams.GetFloat("EdgeMatchingTolerance", "Other") * PIOVER180;
	params.backFaceCulling = cdfParams.GetBool("BackFaceCulling", "Other");
	params.minTexGrabAngle = cdfParams.GetFloat("MinTextureGrabAngle", "Other") * PIOVER180;
	
	
	return params;
}

// SDL - Keyboard and Mouse input control
bool inputsControl(Tracking::Tracker* tracker, float fTimeTracker){
 	int i=0;
 	
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
				case SDLK_e:
					tracker->setEdgesImageFlag( !tracker->getEdgesImageFlag() );
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
					tracker->printStatistics();
					printf("\n	Total tracking time: %.0f ms\n", fTimeTracker*1000);
					break;
				case SDLK_t:
					tracker->textureFromImage();
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






