/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <VisionData.hpp>
#include "Tracker.h"

using namespace cast;
//using namespace VisionData;
using namespace cogx;
using namespace Math;

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

cdl::CASTTime convertTime(double time_sec){
	cdl::CASTTime casttime;
	
	casttime.s = floor(time_sec);
	casttime.us = (time_sec - floor(time_sec)) * 1e6;
	
	return casttime;
}

void transposeMatrix4(float* m1, float* m2){
	m2[0]=m1[0];  m2[1]=m1[4];  m2[2]=m1[8];  m2[3]=m1[12];
	m2[4]=m1[1];  m2[5]=m1[5];  m2[6]=m1[9];  m2[7]=m1[13];
	m2[8]=m1[2];  m2[9]=m1[6];  m2[10]=m1[10]; m2[11]=m1[14];
	m2[12]=m1[3]; m2[13]=m1[7]; m2[14]=m1[11]; m2[15]=m1[15];
}

/* Stuff for adjusting cube 
vector<Model::Vertex> rotateY(vector<Model::Vertex> vertices, float alpha){
	float m[9] = {	cos(alpha), 0, sin(alpha),
									0, 					1, 0,
									-sin(alpha), 0, cos(alpha) };
	mat3 rotY(m);
	
	for(int i=0; i<vertices.size(); i++){
		vec3 v1(vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z);
		vec3 v2 = rotY * v1;
		
		vertices[i].pos.x = v2.x;
		vertices[i].pos.y = v2.y;
		vertices[i].pos.z = v2.z;		
	}

	return vertices;
}

bool compareX(const Model::Vertex& v1, const Model::Vertex& v2){
	return v1.pos.x < v2.pos.x;
}

bool makeCube(Model* rawCube, Model* resultCube){
	float alpha = 3.1415926*0.25;
	float beta = 0.0;
	float gamma = 0.0;
	int i;
	
	
	
	vector<Model::Vertex> vertexlist = rawCube->m_vertexlist;
	
	
	vertexlist = rotateY(vertexlist, alpha);
	
	resultCube->m_vertexlist = vertexlist;
	
	std::sort(vertexlist.begin(), vertexlist.end(), compareX);
		
	printf("m_vertexlist (unsorted):\n");
	for(i=0; i<rawCube->m_vertexlist.size(); i++){
		printf("%f %f %f\n", rawCube->m_vertexlist[i].pos.x, rawCube->m_vertexlist[i].pos.y, rawCube->m_vertexlist[i].pos.z);
	}
	
	printf("vertexlist (sorted):\n");
	for(i=0; i<vertexlist.size(); i++){
		printf("%f %f %f\n", vertexlist[i].pos.x, vertexlist[i].pos.y, vertexlist[i].pos.z);
	}
	
	
	printf("%f %f %f\n", rotX[0], rotX[1], rotX[2]);
	printf("%f %f %f\n", rotX[3], rotX[4], rotX[5]);
	printf("%f %f %f\n", rotX[6], rotX[7], rotX[8]);
	
	

	
	return true;
}

*/









