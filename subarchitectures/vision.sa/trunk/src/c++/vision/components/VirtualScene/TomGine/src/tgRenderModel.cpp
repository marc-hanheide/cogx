
#include "tgRenderModel.h"
#include <GL/gl.h>

using namespace TomGine;

void tgRenderModel::ApplyMaterial(){
	glEnable(GL_LIGHTING);
	glMaterialfv(GL_FRONT,GL_AMBIENT,m_material.ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,m_material.diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,m_material.specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&m_material.shininess);
}

void tgRenderModel::ApplyColor(){
	glColor3f(m_material.color.x, m_material.color.y, m_material.color.z);
}

void tgRenderModel::Material::Apply(){
	glEnable(GL_LIGHTING);
	glMaterialfv(GL_FRONT,GL_AMBIENT,ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&shininess);
}

void tgRenderModel::DrawFaces(){
	DrawFaces(false);
}

void tgRenderModel::DrawFaces(bool lighting){
	int i,j;
	Face* f;
	int v;
	
	if(!lighting){
		ApplyMaterial();
	}else{
		glDisable(GL_LIGHTING);
		glColor3f(m_material.color.x, m_material.color.y, m_material.color.z);
	}	
	
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

void tgRenderModel::DrawPolygons(){
	ApplyMaterial();
	int i,j,v;
	for(i=0; i<(int)m_polygons.size(); i++){
		glBegin(GL_TRIANGLE_FAN);		
			for(j=0; j<(int)m_polygons[i].vertices.size(); j++){
				v = m_polygons[i].vertices[j];
				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
			}
		glEnd();
	}
}

// void tgRenderModel::DrawNormals(float normal_length){	// draw normals
// 	int i,j,v;
// 	Face* f;
// 	
// 	m_pose.Activate();
// 	
// 	glDisable(GL_TEXTURE_2D);
// 	glDisable(GL_LIGHTING);
// 	
// 	glBegin(GL_LINES);
// 	for(i=0; i<m_faces.size(); i++){
// 		f = &m_faces[i];
// 		for(j=0; j<(int)f->vertices.size(); j++){
// 			v = f->vertices[j];
// 			glVertex3f( m_vertices[v].pos.x,
// 									m_vertices[v].pos.y,
// 									m_vertices[v].pos.z );
// 			glVertex3f( m_vertices[v].pos.x + m_vertices[v].normal.x * normal_length,
// 									m_vertices[v].pos.y + m_vertices[v].normal.y * normal_length,
// 									m_vertices[v].pos.z + m_vertices[v].normal.z * normal_length );
// 		}
// 	}
// 	glEnd();
// 	
// 	m_pose.Deactivate();
// 	glColor3f(1.0, 1.0, 1.0);
// }
