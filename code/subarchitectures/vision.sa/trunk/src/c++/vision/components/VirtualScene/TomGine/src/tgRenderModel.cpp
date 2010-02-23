
#include "tgRenderModel.h"
#include <GL/gl.h>

using namespace TomGine;

void tgRenderModel::ApplyMaterial(Material mat){
	glMaterialfv(GL_FRONT,GL_AMBIENT,mat.ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,mat.diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,mat.specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&mat.shininess);
}

void tgRenderModel::Material::Apply(){
	glMaterialfv(GL_FRONT,GL_AMBIENT,ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&shininess);
}

void tgRenderModel::DrawFaces(){
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
			printf("[tgModel::printInfo()] Face has %d vertices (supported: 3 or 4\n", (int)f->vertices.size());
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

void tgRenderModel::DrawNormals(float normal_length){	// draw normals
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
