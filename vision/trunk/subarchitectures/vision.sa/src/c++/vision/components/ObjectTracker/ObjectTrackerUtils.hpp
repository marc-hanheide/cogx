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
	for(i=0; i<geom->vertices.size(); i++){
		Model::Vertex v;
		v.pos.x = geom->vertices[i].pos.x;
		v.pos.y = geom->vertices[i].pos.y;
		v.pos.z = geom->vertices[i].pos.z;
		v.normal.x = geom->vertices[i].normal.x;
		v.normal.y = geom->vertices[i].normal.y;
		v.normal.z = geom->vertices[i].normal.z;
		v.texCoord.x = geom->vertices[i].texCoord.x;
		v.texCoord.y = geom->vertices[i].texCoord.y;
		model->m_vertexlist.push_back(v);
		//printf("Vertex: %f %f %f\n", v.pos.x, v.pos.y, v.pos.z);
	}
	
	// Parse through faces and store content in Model
	for(i=0; i<geom->faces.size(); i++){
		Model::Face f;
		f.v = geom->faces[i].vertices;	
		model->m_facelist.push_back(f);	
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
