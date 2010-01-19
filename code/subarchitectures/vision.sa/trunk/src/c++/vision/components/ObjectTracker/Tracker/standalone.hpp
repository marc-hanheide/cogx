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

static const double REAL_ONE  = ((double)1.0);

// *************************************************************************************
// Parameters
struct CameraParameters {
  // image dimension
  int width;
  int height;
  // Instrinsic parameters:
  // entries of the camera matrix
  double fx;
  double fy;
  double cx;
  double cy;
  // radial distortion parameters
  double k1;
  double k2;
  double k3;
  // tangential distortion parameters
  double p1;
  double p2;
  // extrinsic parameters: 3D pose of camera w.r.t. world
  mat3 rot;
  vec3 pos;
//   Pose3 pose;
};

struct Parameters{
	CameraParameters 		camParams;
	Particle 						constraints;
	
	int									mode;
	int 								recursions;
	int 								particles;
	
	std::string 				modelPath;
	std::string 				texturePath;
	std::string 				shaderPath;
	
	float								edgeMatchingTol;
	bool								backFaceCulling;
	Particle						initialParticle;
	float								minTexGrabAngle;
};

// *************************************************************************************
// Vector/Matrix Conversions
void fromAngleAxis(mat3 &m, double angle, const vec3& axis)
{
  double s = sin(angle), c = cos(angle);
  double v = REAL_ONE - c, x = axis.x*v, y = axis.y*v, z = axis.z*v;

  m[0] = axis.x*x + c;
  m[1] = axis.x*y - axis.z*s;
  m[2] = axis.x*z + axis.y*s;

  m[3] = axis.y*x + axis.z*s;
  m[4] = axis.y*y + c;
  m[5] = axis.y*z - axis.x*s;

	m[6] = axis.z*x - axis.y*s;
 	m[7] = axis.z*y + axis.x*s;
  m[8] = axis.z*z + c;
}

// creates rotation matrix from rotation vector (= axis plus angle).
void fromRotVector(mat3 &m, const vec3& r)
{
  vec3 axis(r);
  double angle = axis.normalize();
  fromAngleAxis(m, angle, axis);
}

// *************************************************************************************
// Converts Video::CameraParameters from Video::Image of VideoServer to 
// Extrinsic- and Intrinsic- Matrix of OpenGL
// zNear and zFar describe the near and far z values of the clipping plane
void loadCameraParameters(Camera& camera, CameraParameters camPars, float zNear, float zFar){
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
	mat3 R = camPars.rot;
	vec3 t = camPars.pos;
	mat4 extrinsic;
	extrinsic[0]=R[0];	extrinsic[4]=R[1];	extrinsic[8]=R[2];		extrinsic[12]=0.0;
	extrinsic[1]=R[3];	extrinsic[5]=R[4];	extrinsic[9]=R[5];		extrinsic[13]=0.0;	
	extrinsic[2]=R[6];	extrinsic[6]=R[7];	extrinsic[10]=R[8];	extrinsic[14]=0.0;	
	extrinsic[3]=0.0;		extrinsic[7]=0.0;		extrinsic[11]=0.0;		extrinsic[15]=1.0;
	extrinsic = extrinsic.transpose();											// R^T
	vec4 tp = -(extrinsic * vec4(t.x, t.y, t.z, 1.0));			// -R^T*t
	extrinsic[12]=tp.x; extrinsic[13]=tp.y; extrinsic[14]=tp.z;
	extrinsic = cv2gl * extrinsic;
	
	// set camera parameters
	camera.SetViewport(camPars.width,camPars.height);
	camera.SetZRange(zNear, zFar);
	camera.SetIntrinsic(intrinsic);
	camera.SetExtrinsic(extrinsic);
	camera.SetPos(camPars.pos.x, camPars.pos.y, camPars.pos.z);
}

// *************************************************************************************
// Load INI File
Parameters LoadParametersFromINI(const char* filename){
	
	Parameters params;
	
	CDataFile cdfParams;
	cdfParams.Load(filename);
	
	// Camera Parameters
	params.camParams.width = cdfParams.GetInt("width", "CameraParameters");
	params.camParams.height = cdfParams.GetInt("height", "CameraParameters");
	params.camParams.fx = cdfParams.GetFloat("fx", "CameraParameters");
	params.camParams.fy = cdfParams.GetFloat("fy", "CameraParameters");
	params.camParams.cx = cdfParams.GetFloat("cx", "CameraParameters");
	params.camParams.cy = cdfParams.GetFloat("cy", "CameraParameters");
	params.camParams.k1 = cdfParams.GetFloat("k1", "CameraParameters");
	params.camParams.k2 = cdfParams.GetFloat("k2", "CameraParameters");
	params.camParams.k3 = cdfParams.GetFloat("k3", "CameraParameters");
	params.camParams.p1 = cdfParams.GetFloat("p1", "CameraParameters");
	params.camParams.p2 = cdfParams.GetFloat("p2", "CameraParameters");
	params.camParams.pos.x = cdfParams.GetFloat("pose.pos.x", "CameraParameters");
	params.camParams.pos.y = cdfParams.GetFloat("pose.pos.y", "CameraParameters");
	params.camParams.pos.z = cdfParams.GetFloat("pose.pos.z", "CameraParameters");
	vec3 vRot;
	vRot.x = cdfParams.GetFloat("pose.rot.x", "CameraParameters");
	vRot.y = cdfParams.GetFloat("pose.rot.y", "CameraParameters");
	vRot.z = cdfParams.GetFloat("pose.rot.z", "CameraParameters");
	fromRotVector(params.camParams.rot, vRot);
	
	// Constraints
	params.constraints.rp.x = cdfParams.GetFloat("rp.x", "Constraints");
	params.constraints.rp.y = cdfParams.GetFloat("rp.y", "Constraints");
	params.constraints.rp.z = cdfParams.GetFloat("rp.z", "Constraints");
	params.constraints.t.x 	= cdfParams.GetFloat("t.x", "Constraints");
	params.constraints.t.y 	= cdfParams.GetFloat("t.y", "Constraints");
	params.constraints.t.z 	= cdfParams.GetFloat("t.z", "Constraints");
	params.constraints.tp.x = cdfParams.GetFloat("tp.x", "Constraints");
	params.constraints.tp.y = cdfParams.GetFloat("tp.y", "Constraints");
	params.constraints.tp.z = cdfParams.GetFloat("tp.z", "Constraints");
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
	params.initialParticle.t.x = cdfParams.GetFloat("InitialParticle.t.x", "Other");
	params.initialParticle.t.y = cdfParams.GetFloat("InitialParticle.t.y", "Other");
	params.initialParticle.t.z = cdfParams.GetFloat("InitialParticle.t.z", "Other");
	params.minTexGrabAngle = cdfParams.GetFloat("MinTextureGrabAngle", "Other") * PIOVER180;
	
	
	return params;
}

// *************************************************************************************
// Input Controls
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
					break;
				case SDLK_t:
					tracker->textureFromImage();
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





