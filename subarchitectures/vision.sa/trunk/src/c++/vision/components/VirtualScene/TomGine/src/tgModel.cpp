//CPP:
//			Title:			class tgModel
//			File:				tgModel.h
//
//			Function:		Representation for geometry using vertices and faces
//
//			Author:			Thomas Mörwald
//			Date:				15.12.2009
// ----------------------------------------------------------------------------

#include "tgModel.h"
#include <GL/gl.h>

tgModel::tgModel(){

}

void tgModel::ApplyMaterial(Material mat){
	glMaterialfv(GL_FRONT,GL_AMBIENT,mat.ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,mat.diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,mat.specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&mat.shininess);
}

void tgModel::DrawFaces(){
	ApplyMaterial(m_material);
	int i,j;
	Face* f;
	int v;
	
	m_pose.Activate();
	
	for(i=0; i<(int)m_faces.size(); i++){
		f = &m_faces[i];
		
		if(f->vertices.size() == 3)
			glBegin(GL_TRIANGLES);
		else if(f->vertices.size() == 4)
			glBegin(GL_QUADS);
		else{
			printf("[tgModel::printInfo()] Warning, no suitable face format\n");
			printf("[tgModel::printInfo()] Face has %d vertices (supported: 3 or 4\n", f->vertices.size());
			m_pose.Deactivate();
			return;
		}
		
			for(j=0; j<(int)f->vertices.size(); j++){
				v = f->vertices[j];
				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
			}
			
		glEnd();
	}

	m_pose.Deactivate();
}

void tgModel::DrawNormals(float normal_length){	// draw normals
	int i,j,v;
	Face* f;
	
	m_pose.Activate();
	
	glDisable(GL_TEXTURE_2D);
	glColor3f(0.0, 0.0, 1.0);
	
	glBegin(GL_LINES);
	for(i=0; i<m_faces.size(); i++){
		f = &m_faces[i];
		for(j=0; j<(int)f->vertices.size(); j++){
			v = f->vertices[j];
			glVertex3f( m_vertices[v].pos.x,
									m_vertices[v].pos.y,
									m_vertices[v].pos.z );
			glVertex3f( m_vertices[v].pos.x + m_vertices[v].normal.x * normal_length,
									m_vertices[v].pos.y + m_vertices[v].normal.y * normal_length,
									m_vertices[v].pos.z + m_vertices[v].normal.z * normal_length );
		}
	}
	glEnd();
	
	m_pose.Deactivate();
	glColor3f(1.0, 1.0, 1.0);
}

void tgModel::ComputeNormals(){

	int i,j;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n;
	
	// calculate vertex normals using the face normal
	for(i=0; i<(int)m_faces.size(); i++){
		f = &m_faces[i];
		
		//if(f->v.size() == 3){ // this is because of some bug in Blender flipping normals of triangles
		
			v0 = vec3(m_vertices[f->vertices[0]].pos.x, m_vertices[f->vertices[0]].pos.y, m_vertices[f->vertices[0]].pos.z);
			v1 = vec3(m_vertices[f->vertices[1]].pos.x, m_vertices[f->vertices[1]].pos.y, m_vertices[f->vertices[1]].pos.z);
			v2 = vec3(m_vertices[f->vertices[2]].pos.x, m_vertices[f->vertices[2]].pos.y, m_vertices[f->vertices[2]].pos.z);
			e1 = v1 - v0;
			e2 = v2 - v0;
			
			n.cross(e1,e2);
			n.normalize();
			f->normal = vec3(n);
			for(j=0; j<(int)f->vertices.size(); j++){
				m_vertices[f->vertices[j]].normal.x = n.x;
				m_vertices[f->vertices[j]].normal.y = n.y;
				m_vertices[f->vertices[j]].normal.z = n.z;
			}
		//}	
	}
}

void tgModel::PrintInfo(){
	int i,j;
	Face* f;
	int v;
	
	for(i=0; i<(int)m_faces.size(); i++){
		f = &m_faces[i];
		
		printf("Face [%d]:\n",i);
		
		for(j=0; j<(int)f->vertices.size(); j++){
			v = f->vertices[j];
			printf("  Vertex [%d]: %f %f %f, %f %f %f\n",i,m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z, m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
		}
	}
}
