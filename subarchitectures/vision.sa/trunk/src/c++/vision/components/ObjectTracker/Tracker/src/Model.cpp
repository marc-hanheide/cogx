
#include "Model.h"

using namespace Tracking;

// Compute normal vectors of vertices
void Model::computeNormals(){
	int i,j;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n;
	
	// calculate vertex normals using the face normal
	for(i=0; i<(int)m_facelist.size(); i++){
		f = &m_facelist[i];
		
		v0 = vec3(m_vertexlist[f->v[0]].pos.x, m_vertexlist[f->v[0]].pos.y, m_vertexlist[f->v[0]].pos.z);
		v1 = vec3(m_vertexlist[f->v[1]].pos.x, m_vertexlist[f->v[1]].pos.y, m_vertexlist[f->v[1]].pos.z);
		v2 = vec3(m_vertexlist[f->v[2]].pos.x, m_vertexlist[f->v[2]].pos.y, m_vertexlist[f->v[2]].pos.z);
		e1 = v1 - v0;
		e2 = v2 - v0;
		
		n.cross(e1,e2);
		n.normalize();
		f->normal = vec3(n);
		for(j=0; j<(int)m_facelist[i].v.size(); j++){
			m_vertexlist[f->v[j]].normal.x = n.x;
			m_vertexlist[f->v[j]].normal.y = n.y;
			m_vertexlist[f->v[j]].normal.z = n.z;
		}
	}
}

// Compute normal vectors of faces
void Model::computeFaceNormals(){
	int i,j;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n;
	
	// calculate vertex normals using the face normal
	for(i=0; i<(int)m_facelist.size(); i++){
		f = &m_facelist[i];
		
		v0 = vec3(m_vertexlist[f->v[0]].pos.x, m_vertexlist[f->v[0]].pos.y, m_vertexlist[f->v[0]].pos.z);
		v1 = vec3(m_vertexlist[f->v[1]].pos.x, m_vertexlist[f->v[1]].pos.y, m_vertexlist[f->v[1]].pos.z);
		v2 = vec3(m_vertexlist[f->v[2]].pos.x, m_vertexlist[f->v[2]].pos.y, m_vertexlist[f->v[2]].pos.z);
		e1 = v1 - v0;
		e2 = v2 - v0;
		
		n.cross(e1,e2);
		n.normalize();
		m_facelist[i].normal = vec3(n);
	}
}

void Model::print(){
	int i,j;
	
	printf("Model:\n");
	for(i=0; i<(int)m_vertexlist.size(); i++){
		Vertex v = m_vertexlist[i];
		printf("Vertex %i: %f %f %f, %f %f %f, %f %f\n", i, v.pos.x, v.pos.y, v.pos.z, v.normal.x, v.normal.y, v.normal.z, v.texCoord.x, v.texCoord.y);
	}
	for(i=0; i<(int)m_facelist.size(); i++){
		printf("Face %i: ",i);
		for(j=0; j<(int)m_facelist[i].v.size(); j++){
			printf("%i ", m_facelist[i].v[j]);
		}
		printf("%f %f %f\n", m_facelist[i].normal.x, m_facelist[i].normal.y, m_facelist[i].normal.z);
	}
}

void Model::clear(){
	m_vertexlist.clear();
	m_facelist.clear();
}