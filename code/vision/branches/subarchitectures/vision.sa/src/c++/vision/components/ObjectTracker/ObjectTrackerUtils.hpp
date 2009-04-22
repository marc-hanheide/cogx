/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <VisionData.hpp>

namespace cast{

using namespace VisionData;

bool convert_GeometryModel_to_ModelData(GeometryModelPtr gm, ModelData* md){
	int i,j;
	
	// Check if model structure is empty
	if(!gm){
		printf("no geometry found\n");
		return false;
	}
	
	// Convert VisualObject::GeometryModel to ObjectTracker::ModelData
	md->num_vertices = gm->vertices.size();
	md->num_faces = gm->faces.size();
	
	// Allocate memory for vertices and faces
	md->m_vertexlist = (ModelData::Vertex*)malloc(sizeof(ModelData::Vertex) * md->num_vertices);
	md->m_facelist = (ModelData::Face*)malloc(sizeof(ModelData::Face) * md->num_faces);
	
	// Parse through vertices and store content in ModelData
	for(i=0; i<md->num_vertices; i++){
		md->m_vertexlist[i].x = gm->vertices[i].pos.x;
		md->m_vertexlist[i].y = gm->vertices[i].pos.y;
		md->m_vertexlist[i].z = gm->vertices[i].pos.z;
		md->m_vertexlist[i].nx = gm->vertices[i].normal.x;
		md->m_vertexlist[i].ny = gm->vertices[i].normal.y;
		md->m_vertexlist[i].nz = gm->vertices[i].normal.z;
		md->m_vertexlist[i].s = gm->vertices[i].texCoord.x;
		md->m_vertexlist[i].t = gm->vertices[i].texCoord.y;
	}
	
	// Parse through faces and store content in ModelData
	for(i=0; i<md->num_faces; i++){
		md->m_facelist[i].nverts = gm->faces[i].vertices.size();
		md->m_facelist[i].v = (unsigned int*)malloc(sizeof(unsigned int) * md->m_facelist[i].nverts);
		for(j=0; j<md->m_facelist[i].nverts; j++)
			md->m_facelist[i].v[j] = gm->faces[i].vertices[j];		
	}
	
	return true;
}

}