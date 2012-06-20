
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

void tgModel::DrawTriangleFan(){
	int i,j,v;
	for(i=0; i<(int)m_trianglefans.size(); i++){
		glBegin(GL_TRIANGLE_FAN);		
			for(j=0; j<(int)m_trianglefans[i].vertices.size(); j++){
				v = m_trianglefans[i].vertices[j];
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
	for(i=0; i<(int)m_lineloops.size(); i++){
		glBegin(GL_LINE_LOOP);
		for(j=0; j<(int)m_lineloops[i].points.size(); j++){
				p = m_lineloops[i].points[j];
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

void tgModel::ComputeFaceNormals(){

	int i,j;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n;
	
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
}

void tgModel::ComputeQuadstripNormals(){
	int i,j,s;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n, n1, n2;
	
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
			v1 = vec3(m_vertices[f->vertices[(j+s-1)%s]].pos.x, m_vertices[f->vertices[(j+s-1)%s]].pos.y, m_vertices[f->vertices[(j+s-1)%s]].pos.z);
			v2 = vec3(m_vertices[f->vertices[(j+s-2)%s]].pos.x, m_vertices[f->vertices[(j+s-2)%s]].pos.y, m_vertices[f->vertices[(j+s-2)%s]].pos.z);
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
	m_quadstrips.clear();
	m_lines.clear();
	m_lineloops.clear();
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

bool PointOnSameSide(vec3 p1, vec3 p2, vec3 a, vec3 b){
	vec3 cp1, cp2;
	cp1.cross(b-a, p1-a);
	cp2.cross(b-a, p2-a);
	if( (cp1*cp2) >= 0.0)
		return true;
		
	return false;
}

void tgModel::TriangulatePolygon(std::vector<vec3> points){
	int i,s;
	int idx = m_vertices.size();
	bool pointInTriangle;
	bool pointIsConvex;
	vec3 e1, e2, n, poly_normal;
	vec3 v0, v1, v2;
	Vertex v;
	Face f;
	std::vector<Vertex> vertices;
	std::vector<Vertex>::iterator v_act, v_pre, v_post, v_in;
	
	s=points.size();
	if(s<3){
		printf("[tgModel::TriangulatePolygon] Warning to few points for polygon: %d\n", s);
		return;
	}
	
	for(i=0; i<s; i++){
		// Calculate normal
		v0 = points[i];
		v1 = points[(i+1)%s];
		v2 = points[(i+s-1)%s];
		e1 = v1-v0;
		e2 = v2-v0;
		e1.normalize();
		e2.normalize();
		n.cross(e1,e2);
		poly_normal = poly_normal + n;	// polygon normal = sum of all normals
		v.pos = v0;
		vertices.push_back(v);
	}
	
	poly_normal.normalize();	// normalize polygon normal
	v_act = vertices.begin();
	while(vertices.size()>2){

		if(v_act==vertices.end()-1){
			v_pre		= v_act-1;
			v_post	= vertices.begin();
		}else if(v_act==vertices.begin()){
			v_pre		= vertices.end()-1;
			v_post	= v_act+1;
		}else{
			v_pre 	= v_act-1;
			v_post	= v_act+1;
		}
		
		// Test if triangle is convex
		v0 = (*v_act).pos;
		v1 = (*v_post).pos;
		v2 = (*v_pre).pos;
		e1 = v1-v0;
		e2 = v2-v0;
		e1.normalize();
		e2.normalize();
		(*v_act).normal.cross(e1,e2);
		(*v_act).normal.normalize();
		
		if((*v_act).normal * poly_normal > 0.0){
			pointIsConvex = true;
		}
		
			// Test if any other point of remaining polygon lies in this triangle
			pointInTriangle = false;
		if(pointIsConvex){
			pointInTriangle = false;
			for(v_in=vertices.begin(); v_in<vertices.end() && vertices.size()>3; v_in++){
				if(v_in!=v_act && v_in!=v_pre && v_in!=v_post){
					if( PointOnSameSide((*v_in).pos, (*v_post).pos, (*v_act).pos, (*v_pre).pos) &&
							PointOnSameSide((*v_in).pos, (*v_act).pos, (*v_pre).pos, (*v_post).pos) &&
							PointOnSameSide((*v_in).pos, (*v_pre).pos, (*v_post).pos, (*v_act).pos) )
					{
						pointInTriangle = true;
					}
				}
			}
		}
			
		if(pointIsConvex && !pointInTriangle){
			// Generate face
			f.vertices.clear();
			
			(*v_pre).normal = poly_normal;
			(*v_act).normal = poly_normal;
			(*v_post).normal = poly_normal;
			
			m_vertices.push_back(*v_pre); 	f.vertices.push_back(idx); idx++;
			m_vertices.push_back(*v_act); 	f.vertices.push_back(idx); idx++;
			m_vertices.push_back(*v_post); 	f.vertices.push_back(idx); idx++;
			
			f.normal = poly_normal;
			m_faces.push_back(f);
			vertices.erase(v_act);
			
		}else{
			v_act = v_post;
		}
	}
}

