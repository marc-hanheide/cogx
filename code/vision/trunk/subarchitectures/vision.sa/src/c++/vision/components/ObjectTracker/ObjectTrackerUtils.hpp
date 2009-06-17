/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <VisionData.hpp>
#include "Tracker.h"

using namespace cast;
using namespace cogx;
using namespace Math;

// converts a VisionData::GeometryModel to a Tracker Model
bool convertGeometryModel(VisionData::GeometryModelPtr geom, Model* model){
	unsigned int i;
	
	// Check if model structure is empty
	if(!geom){
		printf("[GeometryModel_Converter] no geometry found\n");
		return false;
	}
	
	// Parse through vertices and store content in Model
	Model::Vertex v;
	for(i=0; i<geom->vertices.size(); i++){
		v.pos.x = geom->vertices[i].pos.x;
		v.pos.y = geom->vertices[i].pos.y;
		v.pos.z = geom->vertices[i].pos.z;
		v.normal.x = geom->vertices[i].normal.x;
		v.normal.y = geom->vertices[i].normal.y;
		v.normal.z = geom->vertices[i].normal.z;
		v.texCoord.x = geom->vertices[i].texCoord.x;
		v.texCoord.y = geom->vertices[i].texCoord.y;
		model->m_vertexlist.push_back(v);
		printf("Vertex: %f %f %f, %f %f %f \n", v.pos.x, v.pos.y, v.pos.z, v.normal.x, v.normal.y, v.normal.z);
	}
	
	// Parse through faces and store content in Model
	Model::Face f;
	for(i=0; i<geom->faces.size(); i++){	
		f.v = geom->faces[i].vertices;	
		model->m_facelist.push_back(f);
		//printf("Face: %i %i %i %i\n", f.v[0], f.v[1], f.v[2], f.v[3]);
	}
	
	model->computeEdges();
	
	return true;
}

// converts a Tracker Model to a VisionData::GeometryModel
bool convertTrackerModel(Model* model, VisionData::GeometryModelPtr geom){
	unsigned int i;
	
	if(!model){
		printf("[TrackerModel_Converter] no geometry found\n");
		return false;
	}
	
	// Parse through vertices and store content in Model
	for(i=0; i<model->m_vertexlist.size(); i++){
		VisionData::Vertex v;
		v.pos.x = model->m_vertexlist[i].pos.x;
		v.pos.y = model->m_vertexlist[i].pos.y;
		v.pos.z = model->m_vertexlist[i].pos.z;
		v.normal.x = model->m_vertexlist[i].normal.x;
		v.normal.y = model->m_vertexlist[i].normal.y;
		v.normal.z = model->m_vertexlist[i].normal.z;
		v.texCoord.x = model->m_vertexlist[i].texCoord.x;
		v.texCoord.y = model->m_vertexlist[i].texCoord.y;
		geom->vertices.push_back(v);
	}
	
	// Parse through faces and store content in Model
	for(i=0; i<model->m_facelist.size(); i++){
		VisionData::Face f;
		f.vertices = model->m_facelist[i].v;	
		geom->faces.push_back(f);	
	}
	
	
	return true;
}

// converts a particle (x,y,z,alpha,beta,gamma) to a pose (R, t) 
bool convertParticle2Pose(Particle& particle, Pose3& pose){
	
	float rot[9];
	float pos[3];
	
	particle.getPose(rot, pos);
	
	pose.rot.m00 = rot[0]; pose.rot.m01 = rot[1]; pose.rot.m02 = rot[2];
	pose.rot.m10 = rot[3]; pose.rot.m11 = rot[4]; pose.rot.m12 = rot[5];
	pose.rot.m20 = rot[6]; pose.rot.m21 = rot[7]; pose.rot.m22 = rot[8];
	
	pose.pos.x = pos[0];
	pose.pos.y = pos[1];
	pose.pos.z = pos[2];
	
	return true;
}

// converts a pose (R, t) to a particle (x,y,z,alpha,beta,gamma)
bool convertPose2Particle(Pose3& pose, Particle& particle){
	mat3 rot;
	vec3 pos;
	
	rot[0] = (float)pose.rot.m00; rot[1] = (float)pose.rot.m01; rot[2] = (float)pose.rot.m02;
	rot[3] = (float)pose.rot.m10; rot[4] = (float)pose.rot.m11; rot[5] = (float)pose.rot.m12;
	rot[6] = (float)pose.rot.m20; rot[7] = (float)pose.rot.m21; rot[8] = (float)pose.rot.m22;
	
	pos.x = pose.pos.x;
	pos.y = pose.pos.y;
	pos.z = pose.pos.z;
	
	Particle p(rot, pos);
	
	particle = p;
	
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
void loadCameraParameters(Camera* camera, Video::CameraParameters camPars, float zNear, float zFar){
	// intrinsic parameters
	float fx = 2.0*camPars.fx / camPars.width;					// scale range from [0 ... 640] to [0 ... 2]
  float fy = 2.0*camPars.fy / camPars.height;					// scale range from [0 ... 480] to [0 ...-2]
  float cx = 1.0-(2.0*camPars.cx / camPars.width);		// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1]
  float cy = (2.0*camPars.cy / camPars.height)-1.0;		// flip and move coordinates from top to middle of image: [0 ...-2] -> [-1 ... 1]
  float z1 = (zFar+zNear)/(zNear-zFar);								// entries for clipping planes
  float z2 = 2*zFar*zNear/(zNear-zFar);
  
  // intrinsic matrix
  mat4 intrinsic;
  intrinsic[0]=fx;	intrinsic[1]=0;		intrinsic[2]=0;		intrinsic[3]=0;
  intrinsic[4]=0;		intrinsic[5]=fy;	intrinsic[6]=0;		intrinsic[7]=0;
  intrinsic[8]=cx;	intrinsic[9]=cy;	intrinsic[10]=z1;	intrinsic[11]=-1;
  intrinsic[12]=0;	intrinsic[13]=0;	intrinsic[14]=z2;	intrinsic[15]=0;
  
  // computer vision coordinates to OpenGL coordinates transform (rotate 180° about x-axis)
  mat4 cv2gl;
  cv2gl[0]=1.0;  cv2gl[1]=0.0;  cv2gl[2]=0.0;   cv2gl[3]=0.0;  
	cv2gl[4]=0.0;  cv2gl[5]=-1.0; cv2gl[6]=0.0;   cv2gl[7]=0.0;  
	cv2gl[8]=0.0;  cv2gl[9]=0.0;  cv2gl[10]=-1.0; cv2gl[11]=0.0;  
	cv2gl[12]=0.0; cv2gl[13]=0.0; cv2gl[14]=0.0;  cv2gl[15]=1.0;  
	
	// extrinsic parameters
	cogx::Math::Matrix33 R = camPars.pose.rot;
	cogx::Math::Vector3 t = camPars.pose.pos;
	mat4 extrinsic;
	extrinsic[0]=R.m00;	extrinsic[1]=R.m01;	extrinsic[2]=R.m02;		extrinsic[3]=0.0;
	extrinsic[4]=R.m10;	extrinsic[5]=R.m11;	extrinsic[6]=R.m12;		extrinsic[7]=0.0;	
	extrinsic[8]=R.m20;	extrinsic[9]=R.m21;	extrinsic[10]=R.m22;	extrinsic[11]=0.0;	
	extrinsic[12]=0.0;	extrinsic[13]=0.0;	extrinsic[14]=0.0;		extrinsic[15]=1.0;
	vec4 tp = -(extrinsic * vec4(t.x*0.001, t.y*0.001, t.z*0.001, 1.0));
	extrinsic[12]=tp.x; extrinsic[13]=tp.y; extrinsic[14]=tp.z;
	extrinsic = cv2gl * extrinsic;
	
	// set camera parameters
	camera->SetViewport(camPars.width,camPars.height);
	camera->SetZRange(zNear, zFar);
	camera->SetIntrinsic(intrinsic);
	camera->SetExtrinsic(extrinsic);
}









