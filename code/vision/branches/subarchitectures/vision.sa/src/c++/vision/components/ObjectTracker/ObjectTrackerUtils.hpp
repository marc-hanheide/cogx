/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <VisionData.hpp>
#include "Tracker.h"

namespace cast{

using namespace VisionData;

bool convertGeometryModel(GeometryModelPtr gm, Model* m){
	int i,j;
	
	// Check if model structure is empty
	if(!gm){
		printf("[GeometryModel_Converter] no geometry found\n");
		return false;
	}
	
	// Parse through vertices and store content in Model
	for(i=0; i<gm->vertices.size(); i++){
		Model::Vertex v;
		v.pos.x = gm->vertices[i].pos.x;
		v.pos.y = gm->vertices[i].pos.y;
		v.pos.z = gm->vertices[i].pos.z;
		v.normal.x = gm->vertices[i].normal.x;
		v.normal.y = gm->vertices[i].normal.y;
		v.normal.z = gm->vertices[i].normal.z;
		v.texCoord.x = gm->vertices[i].texCoord.x;
		v.texCoord.y = gm->vertices[i].texCoord.y;
		m->m_vertexlist.push_back(v);
	}
	
	// Parse through faces and store content in Model
	for(i=0; i<gm->faces.size(); i++){
		Model::Face f;
		f.v = gm->faces[i].vertices;	
		m->m_facelist.push_back(f);	
	}
	
	m->computeEdges();
	
	return true;
}

bool convertTrackerModel(Model* m, GeometryModelPtr gm){
	int i,j;
	
	if(!m){
		printf("[TrackerModel_Converter] no geometry found\n");
		return false;
	}
	
	// Parse through vertices and store content in Model
	for(i=0; i<m->m_vertexlist.size(); i++){
		VisionData::Vertex v;
		v.pos.x = m->m_vertexlist[i].pos.x;
		v.pos.y = m->m_vertexlist[i].pos.y;
		v.pos.z = m->m_vertexlist[i].pos.z;
		v.normal.x = m->m_vertexlist[i].normal.x;
		v.normal.y = m->m_vertexlist[i].normal.y;
		v.normal.z = m->m_vertexlist[i].normal.z;
		v.texCoord.x = m->m_vertexlist[i].texCoord.x;
		v.texCoord.y = m->m_vertexlist[i].texCoord.y;
		gm->vertices.push_back(v);
	}
	
	// Parse through faces and store content in Model
	for(i=0; i<m->m_facelist.size(); i++){
		VisionData::Face f;
		f.vertices = m->m_facelist[i].v;	
		gm->faces.push_back(f);	
	}
	
	
	return true;
}

}