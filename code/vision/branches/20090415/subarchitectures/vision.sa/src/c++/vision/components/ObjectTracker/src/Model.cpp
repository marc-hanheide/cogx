
#include "Model.h"



// *** PROTECTED ***

void Model::print(){
	int i,j;
	
	printf("Model:\n");
	for(i=0; i<m_vertexlist.size(); i++){
		Vertex v = m_vertexlist[i];
		printf("Vertex %i: %f %f %f, %f %f %f, %f %f\n", i, v.pos.x, v.pos.y, v.pos.z, v.normal.x, v.normal.y, v.normal.z, v.texCoord.x, v.texCoord.y);
	}
	for(i=0; i<m_facelist.size(); i++){
		printf("Face %i: ",i);
		for(j=0; j<m_facelist[i].v.size(); j++){
			printf("%i ", m_facelist[i].v[j]);
		}
		printf("\n");
	}
	for(i=0; i<m_edgelist.size(); i++){
		printf("Edge %i: %i %i\n", i, m_edgelist[i].start, m_edgelist[i].end);
	}
}

// Tests redundancy of edge
bool Model::isRedundant(Edge* e1, int k){
	Edge* e2;
	vec3 vs, ve;
	
	for(int i=0; i<k; i++){
		e2 = &m_edgelist[i];
		
		// Get Vector between start-start and end-end points of edges
		vs = vec3(	m_vertexlist[e1->start].pos.x - m_vertexlist[e2->start].pos.x,
					m_vertexlist[e1->start].pos.y - m_vertexlist[e2->start].pos.y,
					m_vertexlist[e1->start].pos.z - m_vertexlist[e2->start].pos.z);
		ve = vec3(	m_vertexlist[e1->end].pos.x - m_vertexlist[e2->end].pos.x,
					m_vertexlist[e1->end].pos.y - m_vertexlist[e2->end].pos.y,
					m_vertexlist[e1->end].pos.z - m_vertexlist[e2->end].pos.z);
		// if sum of length between vertices is insignificant then redundancy is detected
		if(vs.length() + ve.length() < 0.01){
			printf("Redundant edge detected: %d %d\n", e1->start, e1->end);
			return true;
		}
		
		// Get Vector between start-end and end-start points of edges
		vs = vec3(	m_vertexlist[e1->start].pos.x - m_vertexlist[e2->end].pos.x,
					m_vertexlist[e1->start].pos.y - m_vertexlist[e2->end].pos.y,
					m_vertexlist[e1->start].pos.z - m_vertexlist[e2->end].pos.z);
		ve = vec3(	m_vertexlist[e1->end].pos.x - m_vertexlist[e2->start].pos.x,
					m_vertexlist[e1->end].pos.y - m_vertexlist[e2->start].pos.y,
					m_vertexlist[e1->end].pos.z - m_vertexlist[e2->start].pos.z);
		// if sum of length between vertices is insignificant then redundancy is detected
		if(vs.length() + ve.length() < 0.01){
			printf("Redundant edge detected: %d %d\n", e1->start, e1->end);
			return true;
		}
						
	}
	return false;
}

// Generate OpenGL display list for edges
void Model::genEdgeDisplayList(){
	edgeDisplayList = glGenLists(1);
    glNewList(edgeDisplayList, GL_COMPILE);
    	glBegin(GL_LINES);
        for(int i=0; i<m_edgelist.size(); i++){
        	glNormal3f( m_vertexlist[m_edgelist[i].end].pos.x - m_vertexlist[m_edgelist[i].start].pos.x,
                        m_vertexlist[m_edgelist[i].end].pos.y - m_vertexlist[m_edgelist[i].start].pos.y,
                        m_vertexlist[m_edgelist[i].end].pos.z - m_vertexlist[m_edgelist[i].start].pos.z );
            glVertex3f(m_vertexlist[m_edgelist[i].start].pos.x, m_vertexlist[m_edgelist[i].start].pos.y, m_vertexlist[m_edgelist[i].start].pos.z);
            glVertex3f(m_vertexlist[m_edgelist[i].end].pos.x, m_vertexlist[m_edgelist[i].end].pos.y, m_vertexlist[m_edgelist[i].end].pos.z);
        }
        glEnd();
    glEndList();
}

// *** PUBLIC ***

Model::Model(){
	m_tex_original = 0;
    m_texture = 0;
    edgeDisplayList = -1;
}

Model::Model(const Model& m){
	m_vertexlist = m.m_vertexlist;
	m_facelist = m.m_facelist;
	m_edgelist = m.m_edgelist;
	
	m_tex_original = m.m_tex_original;
	m_texture = m.m_texture;
}

Model::~Model(){
	if(edgeDisplayList != -1)
    	glDeleteLists(edgeDisplayList,1);
}

Model& Model::operator=(const Model& m2){
	
    // copy content
    m_vertexlist = m2.m_vertexlist;
    m_facelist = m2.m_facelist;
    m_edgelist = m2.m_edgelist;
    
    m_tex_original = m2.m_tex_original;
    m_texture = m2.m_texture; 
}

// Generate Edges from faces
void Model::computeEdges(){
    int i,j,k=0;
    
    // Extract edges from faces
    for(i=0; i<m_facelist.size(); i++){
        for(j=0; j<m_facelist[i].v.size(); j++){
        	Edge e;
            e.start = m_facelist[i].v[j];
            e.end = m_facelist[i].v[(j+1)%m_facelist[i].v.size()];
            if(!isRedundant(&m_edgelist[k], k)){
            	//printf("Edge: %d %d\n", m_edgelist[k].start, m_edgelist[k].end);
            	m_edgelist.push_back(e);
            }
        }
    }
}

// Compute vertex normals
void Model::computeNormals(){
	int i,j;
	Face* f;
	vec3 v0, v1, v2, e1, e2, n;
	
	for(i=0; i<m_facelist.size(); i++){
		f = &m_facelist[i];
		
		v0 = vec3(m_vertexlist[f->v[0]].pos.x, m_vertexlist[f->v[0]].pos.y, m_vertexlist[f->v[0]].pos.z);
		v1 = vec3(m_vertexlist[f->v[1]].pos.x, m_vertexlist[f->v[1]].pos.y, m_vertexlist[f->v[1]].pos.z);
		v2 = vec3(m_vertexlist[f->v[2]].pos.x, m_vertexlist[f->v[2]].pos.y, m_vertexlist[f->v[2]].pos.z);
		e1 = v1 - v0;
		e2 = v2 - v0;
		
		n.cross(e1,e2);
		n.normalize();
		
		for(j=0; j<m_facelist[i].v.size(); j++){
			m_vertexlist[f->v[j]].normal.x = n.x;
			m_vertexlist[f->v[j]].normal.y = n.y;
			m_vertexlist[f->v[j]].normal.z = n.z;
		}		
	}
}

// draws only faces of model
void Model::drawFaces(){
	int i,j;
	Face* f;
	
	if(m_texture){
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		m_texture->bind();
	}
	for(i=0; i<m_facelist.size(); i++){
		f = &m_facelist[i];
		glBegin(GL_QUADS);
			for(j=0; j<m_facelist[i].v.size(); j++){
				glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
				glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
			}
		glEnd();
		
		/*
		glDisable(GL_TEXTURE_2D);
		glBegin(GL_LINES);
			float normal_length = 0.01;
			for(j=0; j<f->size(); j++){
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f( m_vertexlist[f->v[j]].x,
							m_vertexlist[f->v[j]].y,
							m_vertexlist[f->v[j]].z );
				glVertex3f( m_vertexlist[f->v[j]].x + m_vertexlist[f->v[j]].nx * normal_length,
							m_vertexlist[f->v[j]].y + m_vertexlist[f->v[j]].ny * normal_length,
							m_vertexlist[f->v[j]].z + m_vertexlist[f->v[j]].nz * normal_length );				
			}
		glEnd();
		glEnable(GL_TEXTURE_2D);
		*/
	}
	
	
	
	if(m_texture){
		glDisable(GL_TEXTURE_2D);
	}
}

// draws only edges of model
void Model::drawEdges(){
	mat4 mv;
	mat3 rot;
	vec3 v_cam_object;
	float s = -0.001;	// = 1mm
	
	glGetFloatv(GL_MODELVIEW_MATRIX, mv);
	
	//printf("%f %f %f %f\n", mv[0], mv[4], mv[8], mv[12]);
	//printf("%f %f %f %f\n", mv[1], mv[5], mv[9], mv[13]);
	//printf("%f %f %f %f\n", mv[2], mv[6], mv[10], mv[14]);
	//printf("%f %f %f %f\n\n", mv[3], mv[7], mv[11], mv[15]);
	
	rot[0] = mv[0]; rot[1] = mv[4]; rot[2] = mv[8];
	rot[3] = mv[1]; rot[4] = mv[5]; rot[5] = mv[9];
	rot[6] = mv[2]; rot[7] = mv[6]; rot[8] = mv[10];
	
	v_cam_object[0] = mv[12];
	v_cam_object[1] = mv[13];
	v_cam_object[2] = mv[14];
	
	v_cam_object = rot * v_cam_object * s;
	
	glPushMatrix();
		// draw edges slightly closer to camera to avoid edge-flickering
		glTranslatef(v_cam_object[0], v_cam_object[1], v_cam_object[2]);
		if(glIsList(edgeDisplayList))
			glCallList(edgeDisplayList);
		else{
			glBegin(GL_LINES);
				for(int i=0; i<m_edgelist.size(); i++){
					glNormal3f( m_vertexlist[m_edgelist[i].end].pos.x - m_vertexlist[m_edgelist[i].start].pos.x,
								m_vertexlist[m_edgelist[i].end].pos.y - m_vertexlist[m_edgelist[i].start].pos.y,
								m_vertexlist[m_edgelist[i].end].pos.z - m_vertexlist[m_edgelist[i].start].pos.z );
					glVertex3f(m_vertexlist[m_edgelist[i].start].pos.x, m_vertexlist[m_edgelist[i].start].pos.y, m_vertexlist[m_edgelist[i].start].pos.z);
					glVertex3f(m_vertexlist[m_edgelist[i].end].pos.x, m_vertexlist[m_edgelist[i].end].pos.y, m_vertexlist[m_edgelist[i].end].pos.z);
				}
			glEnd();
		}
    glPopMatrix();
}

// set original texture as current
void Model::restoreTexture(){
	m_texture = m_tex_original;
}

