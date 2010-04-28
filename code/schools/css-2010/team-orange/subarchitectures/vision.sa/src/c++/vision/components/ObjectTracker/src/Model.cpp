
#include "Model.h"
#include "Resources.h"


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
bool Model::isRedundant(Edge* e1){
	Edge* e2;
	vec3 vs, ve;
	
	for(int i=0; i<m_edgelist.size(); i++){
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
			//printf("Redundant edge detected: %d %d\n", e1->start, e1->end);
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
			//printf("Redundant edge detected: %d %d\n", e1->start, e1->end);
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
    m_textured = false;
    edgeDisplayList = -1;
}

Model::Model(const Model& m){
	m_vertexlist = m.m_vertexlist;
	m_facelist = m.m_facelist;
	m_edgelist = m.m_edgelist;
	m_passlist = m.m_passlist;
	
	sprintf(m_modelname, "%s", m.m_modelname);
	
	m_tex_original = m.m_tex_original;
	m_texture = m.m_texture;
	m_textured = true;
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
    int i,j;
    
    // Extract edges from faces
    for(i=0; i<m_facelist.size(); i++){
        for(j=0; j<m_facelist[i].v.size(); j++){
        	Edge e;
            e.start = m_facelist[i].v[j];
            e.end = m_facelist[i].v[(j+1)%m_facelist[i].v.size()];
            if(!isRedundant(&e)){
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
	
	// calculate vertex normals using the face normal
	for(i=0; i<m_facelist.size(); i++){
		f = &m_facelist[i];
		
		//if(f->v.size() == 3){ // this is because of some bug in Blender flipping normals of triangles
		
			v0 = vec3(m_vertexlist[f->v[0]].pos.x, m_vertexlist[f->v[0]].pos.y, m_vertexlist[f->v[0]].pos.z);
			v1 = vec3(m_vertexlist[f->v[1]].pos.x, m_vertexlist[f->v[1]].pos.y, m_vertexlist[f->v[1]].pos.z);
			v2 = vec3(m_vertexlist[f->v[2]].pos.x, m_vertexlist[f->v[2]].pos.y, m_vertexlist[f->v[2]].pos.z);
			e1 = v1 - v0;
			e2 = v2 - v0;
			
			n.cross(e1,e2);
			n.normalize();
			f->normal = vec3(n);
			
			for(j=0; j<m_facelist[i].v.size(); j++){
				m_vertexlist[f->v[j]].normal.x = n.x;
				m_vertexlist[f->v[j]].normal.y = n.y;
				m_vertexlist[f->v[j]].normal.z = n.z;
				//printf("%f %f %f\n", n.x, n.y, n.z);
			}
		//}	
	}
	
}

void Model::flipNormals(){
	int i;
	Face* f;
	
	for(i=0; i<m_facelist.size(); i++){
		f = &m_facelist[i];
		f->normal.x = -f->normal.x;
		f->normal.y = -f->normal.y;
		f->normal.z = -f->normal.z;
	}
}

// draws model using rendering passes
void Model::drawPass(Shader* shadeTexturing){
	int p,i,j;
	Face* f;
	vector<int> drawnFaces;
	
	if(m_passlist.empty() || !m_textured){
		//printf("[Model::drawPass] Warning no render pass defined. Rendering using Model::drawFaces()\n");
		if(!m_texture)
			shadeTexturing->unbind();
		drawFaces();
		return;
	}
	
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	
	// Draw render passes (textured)
	for(p=0; p<m_passlist.size(); p++){
		
		// bind texture of pass
		m_passlist[p]->texture->bind();
		
		// set modelview matrix for texture
		shadeTexturing->setUniform("modelviewprojection", m_passlist[p]->modelviewprojection, GL_FALSE);
		
		// parse through faces of pass
		for(i=0; i<m_passlist[p]->f.size(); i++){
			f = &m_facelist[m_passlist[p]->f[i]];
			drawnFaces.push_back(m_passlist[p]->f[i]);
			
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
				printf("[Model::drawFaces] Warning unsupported face structure");
				
				for(j=0; j<f->v.size(); j++){
					glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
					glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
					glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
				}
				
			glEnd();
			
			/*
			// Draw Normals
			glDisable(GL_TEXTURE_2D);
			glBegin(GL_LINES);
				float normal_length = 0.01;
				for(j=0; j<m_facelist[i].v.size(); j++){
					glColor3f(0.0, 0.0, 1.0);
					glVertex3f( m_vertexlist[f->v[j]].pos.x,
								m_vertexlist[f->v[j]].pos.y,
								m_vertexlist[f->v[j]].pos.z );
					glVertex3f( m_vertexlist[f->v[j]].pos.x + m_vertexlist[f->v[j]].normal.x * normal_length,
								m_vertexlist[f->v[j]].pos.y + m_vertexlist[f->v[j]].normal.y * normal_length,
								m_vertexlist[f->v[j]].pos.z + m_vertexlist[f->v[j]].normal.z * normal_length );		
				}
			glEnd();
			glEnable(GL_TEXTURE_2D);
			*/
		}
	}
		
	glDisable(GL_TEXTURE_2D);
	shadeTexturing->unbind();
	
	// Draw remaining faces for contours
	for(i=0; i<m_facelist.size(); i++){
		
		f = &m_facelist[i];
		if(f->max_pixels == 0){
		
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
				printf("[Model::drawFaces] Warning unsupported face structure");
				
			for(j=0; j<f->v.size(); j++){
				glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
				glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
				glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
			}
			
			glEnd();
		}
		/*
		// Draw Normals
		glDisable(GL_TEXTURE_2D);
		glBegin(GL_LINES);
			float normal_length = 0.01;
			for(j=0; j<m_facelist[i].v.size(); j++){
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f( m_vertexlist[f->v[j]].pos.x,
							m_vertexlist[f->v[j]].pos.y,
							m_vertexlist[f->v[j]].pos.z );
				glVertex3f( m_vertexlist[f->v[j]].pos.x + m_vertexlist[f->v[j]].normal.x * normal_length,
							m_vertexlist[f->v[j]].pos.y + m_vertexlist[f->v[j]].normal.y * normal_length,
							m_vertexlist[f->v[j]].pos.z + m_vertexlist[f->v[j]].normal.z * normal_length );		
			}
		glEnd();
		glEnable(GL_TEXTURE_2D);
		*/
	}
	
}

// draws only faces of model
void Model::drawFaces(){
	int i,j;
	Face* f;
	bool drawNormals = false;
	
	
	if(m_texture){
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		m_texture->bind();
	}	
	
	for(i=0; i<m_facelist.size(); i++){
		f = &m_facelist[i];
		
		if(f->v.size() == 3)
			glBegin(GL_TRIANGLES);
		else if(f->v.size() == 4)
			glBegin(GL_QUADS);
		else
			printf("[Model::drawFaces] Warning unsupported face structure");
			
			for(j=0; j<f->v.size(); j++){
				glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
				glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
				//glNormal3f(f->normal.x, f->normal.y, f->normal.z);
				glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
			}
			
		glEnd();
		
		
		//glDisable(GL_TEXTURE_2D);
		if(drawNormals){
			glColor3f(0.0, 0.0, 1.0);
			glBegin(GL_LINES);
				float normal_length = 0.01;
				for(j=0; j<m_facelist[i].v.size(); j++){
					glVertex3f( m_vertexlist[f->v[j]].pos.x,
								m_vertexlist[f->v[j]].pos.y,
								m_vertexlist[f->v[j]].pos.z );
					glVertex3f( m_vertexlist[f->v[j]].pos.x + m_facelist[i].normal.x * normal_length,
								m_vertexlist[f->v[j]].pos.y + m_facelist[i].normal.y * normal_length,
								m_vertexlist[f->v[j]].pos.z + m_facelist[i].normal.z * normal_length );		
				}
			glEnd();
			glColor3f(1.0, 1.0, 1.0);
		}
		//glEnable(GL_TEXTURE_2D);
	}
		
	if(m_texture)
		glDisable(GL_TEXTURE_2D);	
}

// draws face i of model
void Model::drawFace(int i){
	int j;
	Face* f;
	
	if(m_texture){
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		m_texture->bind();
	}
	
	
	f = &m_facelist[i];
	if(f->v.size() == 3)
		glBegin(GL_TRIANGLES);
	else if(f->v.size() == 4)
		glBegin(GL_QUADS);
	else
		printf("[Model::drawFaces] Warning unsupported face structure");
	
	for(j=0; j<f->v.size(); j++){
		glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
		glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
		glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
	}
		
	glEnd();
	
	/*
	glDisable(GL_TEXTURE_2D);
	glBegin(GL_LINES);
		float normal_length = 0.01;
		for(j=0; j<m_facelist[i].v.size(); j++){
			glColor3f(0.0, 0.0, 1.0);
			glVertex3f( m_vertexlist[f->v[j]].pos.x,
						m_vertexlist[f->v[j]].pos.y,
						m_vertexlist[f->v[j]].pos.z );
			glVertex3f( m_vertexlist[f->v[j]].pos.x + m_vertexlist[f->v[j]].normal.x * normal_length,
						m_vertexlist[f->v[j]].pos.y + m_vertexlist[f->v[j]].normal.y * normal_length,
						m_vertexlist[f->v[j]].pos.z + m_vertexlist[f->v[j]].normal.z * normal_length );		
		}
	glEnd();
	glEnable(GL_TEXTURE_2D);
	*/
	
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

// counts pixels of each face
// if pixels of face are > than in any previouse view
//   set update flag = true
vector<int> Model::getFaceUpdateList(Particle* p_max){
	int i, n;
	vector<int> faceUpdateList;
	
	// generate occlusion queries
	unsigned int* queryPixels;		// Occlussion query for counting pixels
	queryPixels = (unsigned int*)malloc( sizeof(unsigned int) * m_facelist.size() );
	glGenQueriesARB(m_facelist.size(), queryPixels);
	
	// count pixels for each face and choose if its texture has to be updated
	p_max->activate();
		
		for(i=0; i<m_facelist.size(); i++){
			glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryPixels[i]);
			drawFace(i);
			glEndQueryARB(GL_SAMPLES_PASSED_ARB);			
		}
		
		for(i=0; i<m_facelist.size(); i++){
			glGetQueryObjectivARB(queryPixels[i], GL_QUERY_RESULT_ARB, &n);
			//printf("pixels[%d]: %d %d\n", i, n, m_facelist[i].max_pixels);
			if(m_facelist[i].max_pixels < n){
				faceUpdateList.push_back(i);
				m_facelist[i].max_pixels = n;
			}
		}
		
	p_max->deactivate();
	
	glDeleteQueriesARB(m_facelist.size(), queryPixels);
	free(queryPixels);
	
	return faceUpdateList;
}

void Model::textureFromImage(unsigned char* image, int width, int height, Particle* p_max){
	int i,j,k, id;
	vec4 texcoords_model;
	vec4 vertex;
	mat4 modelview, projection, modelviewprojection;
	
	vector<int> faceUpdateList = getFaceUpdateList(p_max);
	if(faceUpdateList.empty())
		return;
	
	// add new rendering pass
	Pass* newpass = new Pass;
	
	// query modelview and projection matrix
	p_max->activate();
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
	p_max->deactivate();
	newpass->modelviewprojection = projection * modelview;
	
	// store texture
	newpass->texture->load(image, width, height);
	g_Resources->GetImageProcessor()->flipUpsideDown(newpass->texture, newpass->texture);	
	
	// add faces to pass
	newpass->f = faceUpdateList;
	
	m_passlist.push_back(newpass);
			
	// clean up passes
	vector<int> usedfaces;		// holds faces which are allready in use by another pass
	for(i=(m_passlist.size()-1); i>=0; i--){		// parse through passlist topdown
		Pass* p = m_passlist[i];					// current pass
		bool destroy = true;
		
		for(j=0; j<p->f.size(); j++){				// for each face of pass
			bool face_allready_in_use = false;
			for(k=0; k<usedfaces.size(); k++){
				if(p->f[j] == usedfaces[k])			// compare with each face in usedfaces
					face_allready_in_use = true;
			}
			if(!face_allready_in_use){
				usedfaces.push_back(p->f[j]);
				destroy=false;
			}
		}
		
		if(destroy){
			delete(p);
			m_passlist.erase(m_passlist.begin()+i);
		}
	}	
		
}





