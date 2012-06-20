 /**
 * @file VirtualSceneUtils.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tools for the VirtualScene Component.
 */

#include <VisionData.hpp>
#include "tgModel.h"
#include "tgShapeCreator.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif


bool convertPose2tgPose(cogx::Math::Pose3& pose, TomGine::tgPose& tgpose);
bool convertGeometry2Model(VisionData::GeometryModelPtr geom, TomGine::tgModel& model);
bool convertConvexHullPlane2Model(VisionData::ConvexHullPtr cvhull, TomGine::tgModel& model);
bool convertConvexHullObj2Model(VisionData::OneObj object, TomGine::tgModel& model);
bool convertSOI2Model(VisionData::SOIPtr soi, TomGine::tgModel& model, cogx::Math::Vector3& pos);
void loadCameraParameters(TomGine::tgCamera* camera, Video::CameraParameters camPars, float zNear, float zFar);

// converts a pose (R, t) to a particle (x,y,z,alpha,beta,gamma)
bool convertPose2tgPose(cogx::Math::Pose3& pose, TomGine::tgPose& tgpose){
	TomGine::mat3 rot;
	TomGine::vec3 pos;
	
	rot[0] = (float)pose.rot.m00; rot[3] = (float)pose.rot.m01; rot[6] = (float)pose.rot.m02;
	rot[1] = (float)pose.rot.m10; rot[4] = (float)pose.rot.m11; rot[7] = (float)pose.rot.m12;
	rot[2] = (float)pose.rot.m20; rot[5] = (float)pose.rot.m21; rot[8] = (float)pose.rot.m22;
	
	pos.x = pose.pos.x;
	pos.y = pose.pos.y;
	pos.z = pose.pos.z;

	tgpose.SetPose(rot, pos);
	
	return true;
}

// converts a VisionData::GeometryModel to a Scene Model
bool convertGeometry2Model(VisionData::GeometryModelPtr geom, TomGine::tgModel& model){
	unsigned int i;
	
	// Check if model structure is empty
	if(!geom){
		printf("[VirtualSceneUtils::convertGeometry2Model] Warning: no geometry found\n");
		return false;
	}
	
	if(geom->vertices.empty()){
		printf("[VirtualSceneUtils::convertGeometry2Model] Warning: no vertices found\n");
		return false;
	}

	// Parse through vertices and store content in Model
	TomGine::tgModel::Vertex v;
	for(i=0; i<geom->vertices.size(); i++){
		v.pos.x = geom->vertices[i].pos.x;
		v.pos.y = geom->vertices[i].pos.y;
		v.pos.z = geom->vertices[i].pos.z;
		v.normal.x = geom->vertices[i].normal.x;
		v.normal.y = geom->vertices[i].normal.y;
		v.normal.z = geom->vertices[i].normal.z;
		v.texCoord.x = geom->vertices[i].texCoord.x;
		v.texCoord.y = geom->vertices[i].texCoord.y;
		model.m_vertices.push_back(v);
//		printf("Vertex %d: %f %f %f, %f %f %f, %f %f\n", i, v.pos.x, v.pos.y, v.pos.z, v.normal.x, v.normal.y, v.normal.z, v.texCoord.x, v.texCoord.y);
	}
	
	// Parse through faces and store content in Model
	TomGine::tgModel::Face f;
	for(i=0; i<geom->faces.size(); i++){	
		f.vertices = geom->faces[i].vertices;
		model.m_faces.push_back(f);
// 		printf("Face: %i %i %i %i\n", f.vertices[0], f.vertices[1], f.vertices[2], f.vertices[3]);
	}	
	
	return true;
}

bool convertConvexHullPlane2Model(VisionData::ConvexHullPtr cvhull, TomGine::tgModel& model){
	int i,j;
	int vidx = model.GetVerticesSize();

	if(!cvhull){
		printf("[VirtualSceneUtils::convertConvexHull2Model] Warning: no geometry found\n");
		return false;
	}
	
	TomGine::vec3 p;
	std::vector<TomGine::vec3> points;
	TomGine::tgModel::LineLoop ll;
	for(i=0; i<cvhull->PointsSeq.size(); i++){
		p.x = cvhull->PointsSeq[i].x;
		p.y = cvhull->PointsSeq[i].y;
		p.z = cvhull->PointsSeq[i].z;
		points.push_back(p);	
	}
	model.TriangulatePolygon(points);
	
	return true;
}

bool convertConvexHullObj2Model(VisionData::OneObj object, TomGine::tgModel& model){
	int i,j;
	TomGine::tgModel::Vertex v;
	TomGine::tgModel::Face f;
	TomGine::vec3 p;
	std::vector<TomGine::vec3> points;
	int vidx = model.GetVerticesSize();
		
	// Bottom plane
	points.clear();
	for(j=object.pPlane.size()-1; j>=0; j--){
		p.x = object.pPlane[j].x;
		p.y = object.pPlane[j].y;
		p.z = object.pPlane[j].z;
		points.push_back(p);
	}
	model.TriangulatePolygon(points);
	// Top plane
	points.clear();
	for(j=object.pTop.size()-1; j>=0; j--){
		p.x = object.pTop[j].x;
		p.y = object.pTop[j].y;
		p.z = object.pTop[j].z;
		points.push_back(p);
	}
	model.TriangulatePolygon(points);
	
	// side planes
	vidx = model.GetVerticesSize();
	f.vertices.clear();
	for(j=0; j<object.pTop.size(); j++){
		v.pos.x = object.pTop[j].x;
		v.pos.y = object.pTop[j].y;
		v.pos.z = object.pTop[j].z;
		model.m_vertices.push_back(v);
		f.vertices.push_back(vidx++);
		
		v.pos.x = object.pPlane[j].x;
		v.pos.y = object.pPlane[j].y;
		v.pos.z = object.pPlane[j].z;
		model.m_vertices.push_back(v);
		f.vertices.push_back(vidx++);
	}
	if(object.pTop.size()>2){
		v.pos.x = object.pTop[0].x;
		v.pos.y = object.pTop[0].y;
		v.pos.z = object.pTop[0].z;
		model.m_vertices.push_back(v);
		f.vertices.push_back(vidx++);
		
		v.pos.x = object.pPlane[0].x;
		v.pos.y = object.pPlane[0].y;
		v.pos.z = object.pPlane[0].z;
		model.m_vertices.push_back(v);
		f.vertices.push_back(vidx++);
	}
	model.m_quadstrips.push_back(f);
	model.ComputeQuadstripNormals();
		
	return true;
}

bool convertSOI2Model(VisionData::SOIPtr soi, TomGine::tgModel& model, cogx::Math::Vector3& pos){
	TomGine::tgShapeCreator shapeCreator;
	int i;
	bool exists = false;
	
	if( soi->boundingBox.size.x>0.0 && soi->boundingBox.size.y>0.0 && soi->boundingBox.size.z>0.0){
		shapeCreator.CreateBox(model, soi->boundingBox.size.x, soi->boundingBox.size.y, soi->boundingBox.size.z);
		pos = soi->boundingBox.pos;
	}
	
	if( soi->boundingSphere.rad > 0.0){
		shapeCreator.CreateSphere(model, soi->boundingSphere.rad, 2, ICOSAHEDRON);
		pos = soi->boundingSphere.pos;
	}

	return true;
}


// Converts Video::CameraParameters from Video::Image of VideoServer to 
// Extrinsic- and Intrinsic- Matrix of OpenGL
// zNear and zFar describe the near and far z values of the clipping plane
void loadCameraParameters(TomGine::tgCamera* camera, Video::CameraParameters camPars, float zNear, float zFar){
	// intrinsic parameters
	// transform the coordinate system of computer vision to OpenGL 
	//   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
	//   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
	float fx = 2.0*camPars.fx / camPars.width;					// scale range from [0 ... 640] to [0 ... 2]
  float fy = 2.0*camPars.fy / camPars.height;					// scale range from [0 ... 480] to [0 ... 2]
  float cx = 0.0; // TODO 1.0-(2.0*camPars.cx / camPars.width);		// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float cy = 0.0; // TODO (2.0*camPars.cy / camPars.height)-1.0;		// flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float z1 = (zFar+zNear)/(zNear-zFar);								// entries for clipping planes
  float z2 = 2*zFar*zNear/(zNear-zFar);								// look up for gluPerspective
  
  // intrinsic matrix
  TomGine::mat4 intrinsic;
  intrinsic[0]=fx;	intrinsic[4]=0;		intrinsic[8]=cx;	intrinsic[12]=0;
  intrinsic[1]=0;		intrinsic[5]=fy;	intrinsic[9]=cy;	intrinsic[13]=0;
  intrinsic[2]=0;		intrinsic[6]=0;		intrinsic[10]=z1;	intrinsic[14]=z2;  
  intrinsic[3]=0;		intrinsic[7]=0;		intrinsic[11]=-1;	intrinsic[15]=0;	// last row assigns w=-z which inverts cx and cy at w-division
  
  // computer vision camera coordinates to OpenGL camera coordinates transform 
  // rotate 180° about x-axis
  TomGine::mat4 cv2gl;
  cv2gl[0]=1.0; cv2gl[4]=0.0; 	cv2gl[8]=0.0;   cv2gl[12]=0.0;  
	cv2gl[1]=0.0; cv2gl[5]=-1.0;	cv2gl[9]=0.0;   cv2gl[13]=0.0;  
	cv2gl[2]=0.0; cv2gl[6]=0.0; 	cv2gl[10]=-1.0; cv2gl[14]=0.0;  
	cv2gl[3]=0.0; cv2gl[7]=0.0; 	cv2gl[11]=0.0;  cv2gl[15]=1.0;  
	
	// extrinsic parameters
	// look up comments in tools/hardware/video/src/slice/Video.ice
	// p = R^T*(w - t) = (R^T, -R^T*t) * (w,1)
	cogx::Math::Matrix33 R = camPars.pose.rot;
	cogx::Math::Vector3 t = camPars.pose.pos;
	TomGine::mat4 extrinsic;
	extrinsic[0]=R.m00;	extrinsic[4]=R.m01;	extrinsic[8]=R.m02;		extrinsic[12]=0.0;
	extrinsic[1]=R.m10;	extrinsic[5]=R.m11;	extrinsic[9]=R.m12;		extrinsic[13]=0.0;	
	extrinsic[2]=R.m20;	extrinsic[6]=R.m21;	extrinsic[10]=R.m22;	extrinsic[14]=0.0;	
	extrinsic[3]=0.0;		extrinsic[7]=0.0;		extrinsic[11]=0.0;		extrinsic[15]=1.0;
	extrinsic = extrinsic.transpose();											// R^T
	TomGine::vec4 tp = -(extrinsic * TomGine::vec4(t.x, t.y, t.z, 1.0));			// -R^T*t
	extrinsic[12]=tp.x; extrinsic[13]=tp.y; extrinsic[14]=tp.z;
	extrinsic = cv2gl * extrinsic;
	
	// set camera parameters
	camera->SetViewport(camPars.width,camPars.height);
	camera->SetZRange(zNear, zFar);
	camera->SetIntrinsic(intrinsic);
	camera->SetExtrinsic(extrinsic);  
	camera->SetPos(camPars.pose.pos.x, camPars.pose.pos.y, camPars.pose.pos.z);
}

void addVectorToCenterOfRotation(cogx::Math::Vector3& cor, int& n, cogx::Math::Vector3& vector){
	cor = cor * (float)n;
	cor = cor + vector;
	n++;
	cor = cor / (float)n;
}




