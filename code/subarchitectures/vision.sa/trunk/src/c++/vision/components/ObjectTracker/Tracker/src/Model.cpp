
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
	vec3 n = vec3(0.0,0.0,0.0);
	
	for(i=0; i<(int)m_facelist.size(); i++){
		f = &m_facelist[i];
		n = vec3(0.0,0.0,0.0);
		for(j=0; j<(int)f->v.size(); j++){
			n += m_vertexlist[f->v[j]].normal;
		}
		n.normalize();
		f->normal = vec3(n);
	}
}