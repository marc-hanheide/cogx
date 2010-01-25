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

// static const double REAL_ONE  = ((double)1.0);

// // *************************************************************************************
// // Parameters
// struct CameraParameters {
//   // image dimension
//   int width;
//   int height;
//   // Instrinsic parameters:
//   // entries of the camera matrix
//   double fx;
//   double fy;
//   double cx;
//   double cy;
//   // radial distortion parameters
//   double k1;
//   double k2;
//   double k3;
//   // tangential distortion parameters
//   double p1;
//   double p2;
//   // extrinsic parameters: 3D pose of camera w.r.t. world
//   mat3 rot;
//   vec3 pos;
// //   Pose3 pose;
// };

struct Parameters{
	int width;
	int height;
};

// // *************************************************************************************
// // Vector/Matrix Conversions
// void fromAngleAxis(mat3 &m, double angle, const vec3& axis)
// {
//   double s = sin(angle), c = cos(angle);
//   double v = REAL_ONE - c, x = axis.x*v, y = axis.y*v, z = axis.z*v;
// 
//   m[0] = axis.x*x + c;
//   m[1] = axis.x*y - axis.z*s;
//   m[2] = axis.x*z + axis.y*s;
// 
//   m[3] = axis.y*x + axis.z*s;
//   m[4] = axis.y*y + c;
//   m[5] = axis.y*z - axis.x*s;
// 
// 	m[6] = axis.z*x - axis.y*s;
//  	m[7] = axis.z*y + axis.x*s;
//   m[8] = axis.z*z + c;
// }
// 
// // creates rotation matrix from rotation vector (= axis plus angle).
// void fromRotVector(mat3 &m, const vec3& r)
// {
//   vec3 axis(r);
//   double angle = axis.normalize();
//   fromAngleAxis(m, angle, axis);
// }

// // *************************************************************************************
// // Converts Video::CameraParameters from Video::Image of VideoServer to 
// // Extrinsic- and Intrinsic- Matrix of OpenGL
// // zNear and zFar describe the near and far z values of the clipping plane
// void loadCameraParameters(Camera& camera, CameraParameters camPars, float zNear, float zFar){
// 	// intrinsic parameters
// 	// transform the coordinate system of computer vision to OpenGL 
// 	//   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
// 	//   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
// 	float fx = 2.0*camPars.fx / camPars.width;					// scale range from [0 ... 640] to [0 ... 2]
//   float fy = 2.0*camPars.fy / camPars.height;					// scale range from [0 ... 480] to [0 ... 2]
//   float cx = 1.0-(2.0*camPars.cx / camPars.width);		// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
//   float cy = (2.0*camPars.cy / camPars.height)-1.0;		// flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
//   float z1 = (zFar+zNear)/(zNear-zFar);								// entries for clipping planes
//   float z2 = 2*zFar*zNear/(zNear-zFar);								// look up for gluPerspective
//   
//   // intrinsic matrix
//   mat4 intrinsic;
//   intrinsic[0]=fx;	intrinsic[4]=0;		intrinsic[8]=cx;	intrinsic[12]=0;
//   intrinsic[1]=0;		intrinsic[5]=fy;	intrinsic[9]=cy;	intrinsic[13]=0;
//   intrinsic[2]=0;		intrinsic[6]=0;		intrinsic[10]=z1;	intrinsic[14]=z2;  
//   intrinsic[3]=0;		intrinsic[7]=0;		intrinsic[11]=-1;	intrinsic[15]=0;	// last row assigns w=-z which inverts cx and cy at w-division
//   
//   // computer vision camera coordinates to OpenGL camera coordinates transform 
//   // rotate 180° about x-axis
//   mat4 cv2gl;
//   cv2gl[0]=1.0; cv2gl[4]=0.0; 	cv2gl[8]=0.0;   cv2gl[12]=0.0;  
// 	cv2gl[1]=0.0; cv2gl[5]=-1.0;	cv2gl[9]=0.0;   cv2gl[13]=0.0;  
// 	cv2gl[2]=0.0; cv2gl[6]=0.0; 	cv2gl[10]=-1.0; cv2gl[14]=0.0;  
// 	cv2gl[3]=0.0; cv2gl[7]=0.0; 	cv2gl[11]=0.0;  cv2gl[15]=1.0;  
// 	
// 	// extrinsic parameters
// 	// look up comments in tools/hardware/video/src/slice/Video.ice
// 	// p = R^T*(w - t) = (R^T, -R^T*t) * (w,1)
// 	mat3 R = camPars.rot;
// 	vec3 t = camPars.pos;
// 	mat4 extrinsic;
// 	extrinsic[0]=R[0];	extrinsic[4]=R[1];	extrinsic[8]=R[2];		extrinsic[12]=0.0;
// 	extrinsic[1]=R[3];	extrinsic[5]=R[4];	extrinsic[9]=R[5];		extrinsic[13]=0.0;	
// 	extrinsic[2]=R[6];	extrinsic[6]=R[7];	extrinsic[10]=R[8];	extrinsic[14]=0.0;	
// 	extrinsic[3]=0.0;		extrinsic[7]=0.0;		extrinsic[11]=0.0;		extrinsic[15]=1.0;
// 	extrinsic = extrinsic.transpose();											// R^T
// 	vec4 tp = -(extrinsic * vec4(t.x, t.y, t.z, 1.0));			// -R^T*t
// 	extrinsic[12]=tp.x; extrinsic[13]=tp.y; extrinsic[14]=tp.z;
// 	extrinsic = cv2gl * extrinsic;
// 	
// 	// set camera parameters
// 	camera.SetViewport(camPars.width,camPars.height);
// 	camera.SetZRange(zNear, zFar);
// 	camera.SetIntrinsic(intrinsic);
// 	camera.SetExtrinsic(extrinsic);
// 	camera.SetPos(camPars.pos.x, camPars.pos.y, camPars.pos.z);
// }

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





