
#include "TrackerModel.h"
#include "Resources.h"

using namespace std;
using namespace Tracking;

// *** PUBLIC ***

// Constructors
TrackerModel::TrackerModel(){
	m_tex_original = 0;
	m_texture = 0;
	m_textured = false;
	m_bfc = false;
	
	int id;
	if((id = g_Resources->AddShader("texturing", "texturing.vert", "texturing.frag")) == -1)
		exit(1);
	m_shadeTexturing = g_Resources->GetShader(id);
}

TrackerModel::~TrackerModel(){	
	PassList::iterator it = m_passlist.begin();
	while(it != m_passlist.end()){
		delete(*it);
		it++;
	}
}

TrackerModel& TrackerModel::operator=(const Model& m){
	m_vertexlist = m.m_vertexlist;
	m_facelist = m.m_facelist;
	m_edgelist.clear();
	m_passlist.clear();	
	m_facepixellist.assign(m_facelist.size(), 0);
	m_texturedfaces.clear();
	
	computeEdges();
	Update();
	
	return (*this);
}

// computes, updates
void TrackerModel::computeEdges(){
    int i,j;
    
    // Extract edges from faces
    for(i=0; i<(int)m_facelist.size(); i++){
        for(j=0; j<(int)m_facelist[i].v.size(); j++){
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

void TrackerModel::Update(){
	if(m_facepixellist.size() != m_facelist.size())
		m_facepixellist.assign(m_facelist.size(), 0);
		
	UpdateDisplayLists();
}

// draws, prints
void TrackerModel::print(){
	int i,j;
	
	printf("TrackerModel:\n");
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
	for(i=0; i<(int)m_edgelist.size(); i++){
		printf("Edge %i: %i %i\n", i, m_edgelist[i].start, m_edgelist[i].end);
	}
}

void TrackerModel::drawNormals(){
	glCallList(m_dlNormals);
}

void TrackerModel::drawTexturedFaces(){
	if(!m_passlist.empty()){
		glCallList(m_dlTexturedFaces);
	}		
}

void TrackerModel::drawUntexturedFaces(){
	glCallList(m_dlUntexturedFaces);
}

void TrackerModel::drawPass(){
 	glCallList(m_dlPass);
}

void TrackerModel::drawFaces(){
	if(m_texture)	m_texture->bind();
	
	glCallList(m_dlFaces);

	if(m_texture) glDisable(GL_TEXTURE_2D);	
}

void TrackerModel::drawEdges(){
	glCallList(m_dlEdges);
}

void TrackerModel::drawFace(int i){
	int j;
	Face* f;
	
	if(m_texture){
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		m_texture->bind();
	}
	
	
	f = &m_facelist[i];
	if((int)f->v.size() == 3)
		glBegin(GL_TRIANGLES);
	else if(f->v.size() == 4)
		glBegin(GL_QUADS);
	else
		printf("[TrackerModel::drawFaces] Warning unsupported face structure");
	
	for(j=0; j<(int)f->v.size(); j++){
		glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
		glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
		glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
	}
		
	glEnd();
		
	if(m_texture){
		glDisable(GL_TEXTURE_2D);
	}
}

// counts pixels of each face
// if pixels of face are > than in any previouse view
//   set update flag = true
vector<int> TrackerModel::getFaceUpdateList(Pose* p_max, vec3 view, float minTexGrabAngle){
	int i, n;
	vector<int> faceUpdateList;
	float alpha;
	vec3 vT;
	mat3 mR;
	p_max->getPose(mR, vT);
	
	// generate occlusion queries
	unsigned int* queryPixels;		// Occlussion query for counting pixels
	queryPixels = (unsigned int*)malloc( sizeof(unsigned int) * m_facelist.size() );
	glGenQueriesARB(m_facelist.size(), queryPixels);
	
	// count pixels for each face and choose if its texture has to be updated
	p_max->activate();
		
		for(i=0; i<(int)m_facelist.size(); i++){
			glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryPixels[i]);
			drawFace(i);
			glEndQueryARB(GL_SAMPLES_PASSED_ARB);			
		}
		
		for(i=0; i<(int)m_facelist.size(); i++){
			glGetQueryObjectivARB(queryPixels[i], GL_QUERY_RESULT_ARB, &n);
			
			vec3 vn = mR * m_facelist[i].normal;
			alpha = acos(vn*view);
			
			if((m_facepixellist[i]==0) && (alpha>minTexGrabAngle)){
				faceUpdateList.push_back(i);
				m_facepixellist[i] = n;
			}
		}
		
	p_max->deactivate();
	
	glDeleteQueriesARB(m_facelist.size(), queryPixels);
	free(queryPixels);
	
	return faceUpdateList;
}

void TrackerModel::textureFromImage(Texture* image, int width, int height, Pose* p_max, vec3 view, float minTexGrabAngle){
	int i,j,k;
	vec4 texcoords_model;
	vec4 vertex;
	mat4 modelview, projection, modelviewprojection;
	
	// get faces to update
	vector<int> faceUpdateList = getFaceUpdateList(p_max, view, minTexGrabAngle);
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
	newpass->texture->copyFromTexture(image);
	
	// add faces to pass
	newpass->f = faceUpdateList;
	
	m_passlist.push_back(newpass);
			
	// clean up passes
	m_texturedfaces.clear();		// holds faces which are allready in use by another pass
	for(i=(m_passlist.size()-1); i>=0; i--){		// parse through passlist topdown
		Pass* p = m_passlist[i];					// current pass
		bool destroy = true;
		
		for(j=0; j<(int)p->f.size(); j++){				// for each face of pass
			bool face_allready_in_use = false;
			for(k=0; k<(int)m_texturedfaces.size(); k++){
				if(p->f[j] == m_texturedfaces[k])			// compare with each face in usedfaces
					face_allready_in_use = true;
			}
			if(!face_allready_in_use){
				m_texturedfaces.push_back(p->f[j]);
				destroy=false;
			}
		}
		
		if(destroy){
			delete(p);
			m_passlist.erase(m_passlist.begin()+i);
		}
	}
	
	m_textured = true;
	m_shadeTexturing->bind();
	m_shadeTexturing->setUniform("useTexCoords", false);
	m_shadeTexturing->unbind();
	
	UpdateDisplayLists();
}


// *** PROTECTED ***

// Tests redundancy of edge
bool TrackerModel::isRedundant(Edge* e1){
	Edge* e2;
	vec3 vs, ve;
	
	for(int i=0; i<(int)m_edgelist.size(); i++){
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

// Generate all display lists
void TrackerModel::UpdateDisplayLists(){

	if(glIsList(m_dlTexturedFaces)) 	glDeleteLists(m_dlTexturedFaces, 1);
	if(glIsList(m_dlUntexturedFaces)) glDeleteLists(m_dlUntexturedFaces, 1);
	if(glIsList(m_dlPass)) 						glDeleteLists(m_dlPass, 1);
	if(glIsList(m_dlFaces)) 					glDeleteLists(m_dlFaces, 1);
	if(glIsList(m_dlEdges)) 					glDeleteLists(m_dlEdges, 1);
	if(glIsList(m_dlNormals)) 				glDeleteLists(m_dlNormals, 1);

	m_dlTexturedFaces = glGenLists(1);
	m_dlUntexturedFaces = glGenLists(1);
	m_dlPass = glGenLists(1);
	m_dlFaces = glGenLists(1);
	m_dlEdges = glGenLists(1);
	m_dlNormals = glGenLists(1);

	if(!m_passlist.empty()){
		glNewList(m_dlTexturedFaces, GL_COMPILE);
			genListTexturedFaces();
		glEndList();
		
		glNewList(m_dlPass, GL_COMPILE);
			genListPass();
		glEndList();
	}else{
		glNewList(m_dlPass, GL_COMPILE);
			genListFaces();
		glEndList();
	}
	
	glNewList(m_dlUntexturedFaces, GL_COMPILE);
		genListUntexturedFaces();
	glEndList();
	
	glNewList(m_dlFaces, GL_COMPILE);
		genListFaces();
	glEndList();
	
	glNewList(m_dlEdges, GL_COMPILE);
		genListEdges();
	glEndList();
	
	glNewList(m_dlNormals, GL_COMPILE);
		genListNormals(0.01);
	glEndList();	
}

// Display List generators
void TrackerModel::genListTexturedFaces(){		// draw only textured faces
		
	int p,i,j;
	Face* f;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	for(p=0; p<(int)m_passlist.size(); p++){
		// parse through faces of pass
		for(i=0; i<(int)m_passlist[p]->f.size(); i++){
			f = &m_facelist[m_passlist[p]->f[i]];
			
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
				printf("[TrackerModel::drawFaces] Warning unsupported face structure");
				
			for(j=0; j<(int)f->v.size(); j++){
				glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
				glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
				glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
			}
			glEnd();			
		}//for i
	}//for p
}

void TrackerModel::genListUntexturedFaces(){		// draw only untextured faces
	int i,j;
	Face* f;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	for(i=0; i<(int)m_facelist.size(); i++){
		
		f = &m_facelist[i];
		if(m_facepixellist[i] == 0){
		
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
				printf("[TrackerModel::drawFaces] Warning unsupported face structure");
				
			for(j=0; j<(int)f->v.size(); j++){
				glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
				glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
				glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
			}
			
			glEnd();			
		}//if
	}	//for
}

void TrackerModel::genListPass(){		// draw faces using passlist and shader for texturing (modelviewmatrix)
	int p,i,j;
	Face* f;
	vec2 texCoords;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	glColor3f(1.0,1.0,1.0);

	m_shadeTexturing->bind();
	
	// Draw render passes (textured)
	for(p=0; p<(int)m_passlist.size(); p++){
		
		// bind texture of pass
		m_passlist[p]->texture->bind();
		
		// set modelview matrix for texture
		m_shadeTexturing->setUniform("modelviewprojection", m_passlist[p]->modelviewprojection, GL_FALSE);
		
		// parse through faces of pass
		for(i=0; i<(int)m_passlist[p]->f.size(); i++){
			f = &m_facelist[m_passlist[p]->f[i]];
			
			if(f->v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(f->v.size() == 4)
				glBegin(GL_QUADS);
			else
				printf("[TrackerModel::drawFaces] Warning unsupported face structure");
				
				for(j=0; j<(int)f->v.size(); j++){
					
					vec4 v = vec4(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z, 1.0);
					v = m_passlist[p]->modelviewprojection * v;
					texCoords.x = (v.x/v.w + 1.0) * 0.5;
					texCoords.y = (v.y/v.w + 1.0) * 0.5;
					
					glTexCoord2f(texCoords.x, texCoords.y);
					glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
					glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
				}
				
			glEnd();
		}//for i
	}//for p
		
	glDisable(GL_TEXTURE_2D);
	m_shadeTexturing->unbind();
	
	// Draw remaining faces for contours
	for(i=0; i<(int)m_facelist.size(); i++){
		
		if(m_facepixellist[i] == 0){
		
			if(m_facelist[i].v.size() == 3)
				glBegin(GL_TRIANGLES);
			else if(m_facelist[i].v.size() == 4)
				glBegin(GL_QUADS);
			else
				printf("[TrackerModel::drawFaces] Warning unsupported face structure");
				
			for(j=0; j<(int)m_facelist[i].v.size(); j++){
				glTexCoord2f(m_vertexlist[m_facelist[i].v[j]].texCoord.x, m_vertexlist[m_facelist[i].v[j]].texCoord.y);
				glNormal3f(m_vertexlist[m_facelist[i].v[j]].normal.x, m_vertexlist[m_facelist[i].v[j]].normal.y, m_vertexlist[m_facelist[i].v[j]].normal.z);
				glVertex3f(m_vertexlist[m_facelist[i].v[j]].pos.x, m_vertexlist[m_facelist[i].v[j]].pos.y, m_vertexlist[m_facelist[i].v[j]].pos.z);
// 				printf("%f %f %f\n", m_vertexlist[m_facelist[i].v[j]].normal.x, m_vertexlist[m_facelist[i].v[j]].normal.y, m_vertexlist[m_facelist[i].v[j]].normal.z);
			}
			
			glEnd();
		}//if
	}//for i
}

void TrackerModel::genListFaces(){		// draw all faces of model
	int i,j;
	Face* f;
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);

	for(i=0; i<(int)m_facelist.size(); i++){
		f = &m_facelist[i];
		
		if((int)f->v.size() == 3)
			glBegin(GL_TRIANGLES);
		else if((int)f->v.size() == 4)
			glBegin(GL_QUADS);
		else
			printf("[TrackerModel::drawFaces] Warning unsupported face structure");
			
			for(j=0; j<(int)f->v.size(); j++){
				glTexCoord2f(m_vertexlist[f->v[j]].texCoord.x, m_vertexlist[f->v[j]].texCoord.y);
				glNormal3f(m_vertexlist[f->v[j]].normal.x, m_vertexlist[f->v[j]].normal.y, m_vertexlist[f->v[j]].normal.z);
				glVertex3f(m_vertexlist[f->v[j]].pos.x, m_vertexlist[f->v[j]].pos.y, m_vertexlist[f->v[j]].pos.z);
			}
			
		glEnd();
	}
}

void TrackerModel::genListEdges(){		// draw all edges of model
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

		glBegin(GL_LINES);
			for(int i=0; i<(int)m_edgelist.size(); i++){
				glNormal3f( m_vertexlist[m_edgelist[i].end].pos.x - m_vertexlist[m_edgelist[i].start].pos.x,
										m_vertexlist[m_edgelist[i].end].pos.y - m_vertexlist[m_edgelist[i].start].pos.y,
										m_vertexlist[m_edgelist[i].end].pos.z - m_vertexlist[m_edgelist[i].start].pos.z );
				glVertex3f(m_vertexlist[m_edgelist[i].start].pos.x, m_vertexlist[m_edgelist[i].start].pos.y, m_vertexlist[m_edgelist[i].start].pos.z);
				glVertex3f(m_vertexlist[m_edgelist[i].end].pos.x, m_vertexlist[m_edgelist[i].end].pos.y, m_vertexlist[m_edgelist[i].end].pos.z);
			}
		glEnd();

    glPopMatrix();
}

void TrackerModel::genListNormals(float normal_length){	// draw normals
	int i,j;
	Face* f;
	
	glDisable(GL_TEXTURE_2D);
	glColor3f(0.0, 0.0, 1.0);
	
	glBegin(GL_LINES);
	for(i=0; i<m_facelist.size(); i++){
		f = &m_facelist[i];
		for(j=0; j<(int)f->v.size(); j++){
			glVertex3f( m_vertexlist[f->v[j]].pos.x,
									m_vertexlist[f->v[j]].pos.y,
									m_vertexlist[f->v[j]].pos.z );
			glVertex3f( m_vertexlist[f->v[j]].pos.x + m_vertexlist[f->v[j]].normal.x * normal_length,
									m_vertexlist[f->v[j]].pos.y + m_vertexlist[f->v[j]].normal.y * normal_length,
									m_vertexlist[f->v[j]].pos.z + m_vertexlist[f->v[j]].normal.z * normal_length );
		}
	}
	glEnd();
	
	glColor3f(1.0, 1.0, 1.0);
}



