
#include "tgModel.h"
#include <GL/gl.h>

using namespace TomGine;

void tgModel::DrawFaces(){
	int i,j;
	Face* f;
	int v;
		
	for(i=0; i<(int)m_faces.size(); i++){
		f = &m_faces[i];
		
		if(f->vertices.size() == 3)
			glBegin(GL_TRIANGLES);
		else if(f->vertices.size() == 4)
			glBegin(GL_QUADS);
		else{
			printf("[tgModel::printInfo()] Warning, no suitable face format\n");
			printf("[tgModel::printInfo()] Face has %d vertices (supported: 3 or 4\n", (int)f->vertices.size());
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
}

void tgModel::DrawPolygons(){
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

void tgModel::DrawQuadstrips(){
	int i,j,v;
	for(i=0; i<(int)m_quadstrips.size(); i++){
		glBegin(GL_QUAD_STRIP);		
			for(j=0; j<(int)m_quadstrips[i].vertices.size(); j++){
				v = m_quadstrips[i].vertices[j];
				glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
				glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
				glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
			}
		glEnd();
	}
}

void tgModel::DrawLines(){
	for(int i=0; i<(int)m_lines.size(); i++){
		glBegin(GL_LINES);
			glVertex3f(m_lines[i].start.x, m_lines[i].start.y, m_lines[i].start.z);
			glVertex3f(m_lines[i].end.x, m_lines[i].end.y, m_lines[i].end.z);
		glEnd();
	}
}

void tgModel::DrawLineLoops(){
	int i,j;
	vec3 p;
	for(i=0; i<(int)m_lineloop.size(); i++){
		glBegin(GL_LINE_LOOP);
		for(j=0; j<(int)m_lineloop[i].points.size(); j++){
				p = m_lineloop[i].points[j];
				glVertex3f(p.x, p.y, p.z);
		}
		glEnd();
	}
}

void tgModel::DrawPoints(){
	for(int i=0; i<(int)m_points.size(); i++){
		glBegin(GL_POINTS);
			glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
		glEnd();
	}
}

void tgModel::DrawNormals(float normal_length){	// draw normals
	m_vertices.size();
	int i,j,v;
	Face* f;
		
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glColor3f(0.0, 0.0, 1.0);
	
	glBegin(GL_LINES);
// 	for(i=0; i<m_faces.size(); i++){
// 		f = &m_faces[i];
	
		for(j=0; j<(int)m_vertices.size(); j++){
			glVertex3f( m_vertices[j].pos.x,
									m_vertices[j].pos.y,
									m_vertices[j].pos.z );
			glVertex3f( m_vertices[j].pos.x + m_vertices[j].normal.x * normal_length,
									m_vertices[j].pos.y + m_vertices[j].normal.y * normal_length,
									m_vertices[j].pos.z + m_vertices[j].normal.z * normal_length );
		}
// 	}
	glEnd();
	
	glColor3f(1.0, 1.0, 1.0);
}

void tgModel::ComputeNormals(){

	int i,j,s;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n, n1, n2;
	
	// calculate vertex normals using the face normal
	for(i=0; i<(int)m_faces.size(); i++){
		f = &m_faces[i];
		
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
	}
	
	for(i=0; i<(int)m_polygons.size(); i++){
		f = &m_polygons[i];
		for(j=0; j<(int)f->vertices.size()-2; j++){
			v0 = vec3(m_vertices[f->vertices[j+0]].pos.x, m_vertices[f->vertices[j+0]].pos.y, m_vertices[f->vertices[j+0]].pos.z);
			v1 = vec3(m_vertices[f->vertices[j+1]].pos.x, m_vertices[f->vertices[j+1]].pos.y, m_vertices[f->vertices[j+1]].pos.z);
			v2 = vec3(m_vertices[f->vertices[j+2]].pos.x, m_vertices[f->vertices[j+2]].pos.y, m_vertices[f->vertices[j+2]].pos.z);
			e1 = v1 - v0;
			e2 = v2 - v0;
			
			n.cross(e1,e2);
			n.normalize();
			
			m_vertices[f->vertices[j+0]].normal.x = n.x;
			m_vertices[f->vertices[j+0]].normal.y = n.y;
			m_vertices[f->vertices[j+0]].normal.z = n.z;
			m_vertices[f->vertices[j+1]].normal.x = n.x;
			m_vertices[f->vertices[j+1]].normal.y = n.y;
			m_vertices[f->vertices[j+1]].normal.z = n.z;
			m_vertices[f->vertices[j+2]].normal.x = n.x;
			m_vertices[f->vertices[j+2]].normal.y = n.y;
			m_vertices[f->vertices[j+2]].normal.z = n.z;
		}
	}
	for(i=0; i<(int)m_quadstrips.size(); i++){
		f = &m_quadstrips[i];
		s = (int)f->vertices.size();
		for(j=0; j<(int)s; j++){
				
			v0 = vec3(m_vertices[f->vertices[(j+0)%s]].pos.x, m_vertices[f->vertices[(j+0)%s]].pos.y, m_vertices[f->vertices[(j+0)%s]].pos.z);
			v1 = vec3(m_vertices[f->vertices[(j+1)%s]].pos.x, m_vertices[f->vertices[(j+1)%s]].pos.y, m_vertices[f->vertices[(j+1)%s]].pos.z);
			v2 = vec3(m_vertices[f->vertices[(j+2)%s]].pos.x, m_vertices[f->vertices[(j+2)%s]].pos.y, m_vertices[f->vertices[(j+2)%s]].pos.z);
			e1 = v1 - v0;
			e2 = v2 - v0;
			n1.cross(e1,e2);
			n1.normalize();
			
			v0 = vec3(m_vertices[f->vertices[(j+0)%s]].pos.x, m_vertices[f->vertices[(j+0)%s]].pos.y, m_vertices[f->vertices[(j+0)%s]].pos.z);
			v1 = vec3(m_vertices[f->vertices[(j-1)%s]].pos.x, m_vertices[f->vertices[(j+s-1)%s]].pos.y, m_vertices[f->vertices[(j+s-1)%s]].pos.z);
			v2 = vec3(m_vertices[f->vertices[(j-2)%s]].pos.x, m_vertices[f->vertices[(j+s-2)%s]].pos.y, m_vertices[f->vertices[(j+s-2)%s]].pos.z);
			e1 = v1 - v0;
			e2 = v2 - v0;
			n2.cross(e2,e1);
			n2.normalize();
			
			n = (n1 + n2) * 0.5;
			
			if(j%2) n = n * -1.0;
			
			m_vertices[f->vertices[j]].normal.x = n.x;
			m_vertices[f->vertices[j]].normal.y = n.y;
			m_vertices[f->vertices[j]].normal.z = n.z;
		}
	}
}

void tgModel::Clear(){
	m_vertices.clear();
	m_faces.clear();
	m_polygons.clear();
	m_quadstrips.clear();
	m_lines.clear();
	m_lineloop.clear();
	m_points.clear();
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
