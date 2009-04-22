
#include "Model.h"

ModelData::ModelData(){
	num_vertices = 0;
    num_faces = 0;
    num_edges = 0;
    m_vertexlist = 0;
    m_facelist = 0;
    m_edgelist = 0;
    m_tex_original = 0;
    m_texture = 0;    
}

ModelData::~ModelData(){
	if(m_vertexlist)
        free(m_vertexlist);
    
    if(m_facelist){
    	//if(m_facelist->v){
		//	for(int i=0; i<m_facelist->nverts; i++)        
		//		free(m_facelist->v);
		//}
			
		free(m_facelist);
    }
    
    if(m_edgelist)
        free(m_edgelist);
}

ModelData& ModelData::operator=(const ModelData& md2){
	int i,j;
	num_vertices = md2.num_vertices;
    num_faces = md2.num_faces;
    num_edges = md2.num_edges;
    
    // deep copy
    m_vertexlist = (Vertex*)malloc(sizeof(Vertex) * num_vertices);
    m_facelist = (Face*)malloc(sizeof(Face) * num_faces);
    m_edgelist = (Edge*)malloc(sizeof(Edge) * num_edges);
    
    for(i=0; i<num_vertices; i++)
    	m_vertexlist[i] = md2.m_vertexlist[i];
    
    for(i=0; i<num_faces; i++){
    	m_facelist[i].nverts = md2.m_facelist[i].nverts;
    	m_facelist[i].v = (unsigned int*)malloc(sizeof(unsigned int) * m_facelist[i].nverts);
    	for(j=0; j<m_facelist[i].nverts; j++)
    		m_facelist[i].v[j] = md2.m_facelist[i].v[j];
    }
    	
    for(i=0; i<num_edges; i++)
    	m_edgelist[i] = md2.m_edgelist[i];
    
    // shallow copy (as textures are managed by g_Resources)
    m_tex_original = md2.m_tex_original;
    m_texture = md2.m_texture; 
}

// *** PRIVATE ***

// Generate OpenGL display list for faces
void Model::genFaceDisplayList(){
    faceDisplayList = glGenLists(1);
    
    glNewList(faceDisplayList, GL_COMPILE);
		drawFaces();
    glEndList();
}

// Generate OpenGL display list for edges
void Model::genEdgeDisplayList(){
    edgeDisplayList = glGenLists(1);
    glNewList(edgeDisplayList, GL_COMPILE);
        glBegin(GL_LINES);
        for(int i=0; i<modeldata.num_edges; i++){
            glNormal3f( modeldata.m_vertexlist[modeldata.m_edgelist[i].end].x - modeldata.m_vertexlist[modeldata.m_edgelist[i].start].x,
                        modeldata.m_vertexlist[modeldata.m_edgelist[i].end].y - modeldata.m_vertexlist[modeldata.m_edgelist[i].start].y,
                        modeldata.m_vertexlist[modeldata.m_edgelist[i].end].z - modeldata.m_vertexlist[modeldata.m_edgelist[i].start].z );
            glVertex3f(modeldata.m_vertexlist[modeldata.m_edgelist[i].start].x, modeldata.m_vertexlist[modeldata.m_edgelist[i].start].y, modeldata.m_vertexlist[modeldata.m_edgelist[i].start].z);
            glVertex3f(modeldata.m_vertexlist[modeldata.m_edgelist[i].end].x, modeldata.m_vertexlist[modeldata.m_edgelist[i].end].y, modeldata.m_vertexlist[modeldata.m_edgelist[i].end].z);
        }
        glEnd();
    glEndList();
}

// Generate Edges from faces
void Model::computeEdges(){
    int i,j,k=0;
    ModelData::Edge* tmp_edgelist;
    
    // count total edges in geometry (redundant)
    modeldata.num_edges = 0;
    for(i=0; i<modeldata.num_faces; i++){
        modeldata.num_edges += modeldata.m_facelist[i].nverts;
    }
    modeldata.m_edgelist = (ModelData::Edge*)malloc(sizeof(ModelData::Edge)*modeldata.num_edges);
    
    // Extrect edges from faces
    for(i=0; i<modeldata.num_faces; i++){
        for(j=0; j<modeldata.m_facelist[i].nverts; j++){
            modeldata.m_edgelist[k].start = modeldata.m_facelist[i].v[j];
            modeldata.m_edgelist[k].end = modeldata.m_facelist[i].v[(j+1)%modeldata.m_facelist[i].nverts];
            if(!isRedundant(&modeldata.m_edgelist[k], k)){
            	//printf("Edge: %d %d\n", modeldata.m_edgelist[k].start, modeldata.m_edgelist[k].end);
            	k++;
            }
        }
    }
    
    // resize storage for edgelist
    modeldata.num_edges = k;
    tmp_edgelist = (ModelData::Edge*)malloc(sizeof(ModelData::Edge)*modeldata.num_edges);
    for(i=0; i<modeldata.num_edges;i++){
    	tmp_edgelist[i] = modeldata.m_edgelist[i];
    }
    free(modeldata.m_edgelist);
    modeldata.m_edgelist = tmp_edgelist;
}


void Model::computeNormals(){
	int i,j;
	ModelData::Face* f;
	TM_Vector3 v0, v1, v2, e1, e2, n;
	
	for(i=0; i<modeldata.num_faces; i++){
		f = &modeldata.m_facelist[i];
		v0 = TM_Vector3(modeldata.m_vertexlist[f->v[0]].x, modeldata.m_vertexlist[f->v[0]].y, modeldata.m_vertexlist[f->v[0]].z);
		v1 = TM_Vector3(modeldata.m_vertexlist[f->v[1]].x, modeldata.m_vertexlist[f->v[1]].y, modeldata.m_vertexlist[f->v[1]].z);
		v2 = TM_Vector3(modeldata.m_vertexlist[f->v[2]].x, modeldata.m_vertexlist[f->v[2]].y, modeldata.m_vertexlist[f->v[2]].z);
		e1 = v1 - v0;
		e2 = v2 - v0;
		
		n = e1.cross(e2);
		n.normalize();
		
		for(j=0; j<f->nverts; j++){
			modeldata.m_vertexlist[f->v[j]].nx = n.x;
			modeldata.m_vertexlist[f->v[j]].ny = n.y;
			modeldata.m_vertexlist[f->v[j]].nz = n.z;
		}		
	}
}


// Compute tangent space for each face
/*void Model::computeTBN(){
void Model::computeTBN(){
	Face* f;
	TM_Vector3 p1, p2, p3;
	float u1, u2, u3;
	float v1, v2, v3;
	TM_Vector3 t,b,n;
	
	for(int i=0; i<modeldata.num_faces; i++){
		// for each face
		f = &modeldata.m_facelist[i];
		
		// compute T,B,N (http://jerome.jouvie.free.fr/OpenGl/Lessons/Lesson8.php)
		p1 = TM_Vector3(modeldata.m_vertexlist[f->v[0]].x,modeldata.m_vertexlist[f->v[0]].y,modeldata.m_vertexlist[f->v[0]].z);
		p2 = TM_Vector3(modeldata.m_vertexlist[f->v[1]].x,modeldata.m_vertexlist[f->v[1]].y,modeldata.m_vertexlist[f->v[1]].z);
		p3 = TM_Vector3(modeldata.m_vertexlist[f->v[2]].x,modeldata.m_vertexlist[f->v[2]].y,modeldata.m_vertexlist[f->v[2]].z);
		u1 = modeldata.m_vertexlist[f->v[0]].s;
		u2 = modeldata.m_vertexlist[f->v[1]].s;
		u3 = modeldata.m_vertexlist[f->v[2]].s;
		v1 = modeldata.m_vertexlist[f->v[0]].t;
		v2 = modeldata.m_vertexlist[f->v[1]].t;
		v3 = modeldata.m_vertexlist[f->v[2]].t;
		
		t = ( ((p2-p1)*(v3-v1))-((p3-p1)*(v2-v1)) ) * 1/( ((u2-u1)*(v3-v1))-((v2-v1)*(u3-u1)) );
		t.normalize();
		b = ( ((p2-p1)*(u3-u1))-((p3-p1)*(u2-u1)) ) * 1/( ((v2-v1)*(u3-u1))-((u2-u1)*(v3-v1)) );
		b.normalize();
		n = TM_Vector3::cross(t,b);
		n.normalize();
		
		// store result to face structure
		f->t[0] = t.x; f->t[1] = t.y; f->t[2] = t.z;
		f->b[0] = b.x; f->b[1] = b.y; f->b[2] = b.z;
		f->n[0] = n.x; f->n[1] = n.y; f->n[2] = n.z;
		
		//printf("\nTBN\n");
		//printf("%f %f %f\n", f->t[0], f->b[0], f->n[0]);
		//printf("%f %f %f\n", f->t[1], f->b[1], f->n[1]);
		//printf("%f %f %f\n", f->t[2], f->b[2], f->n[2]);
		//printf("\n");
		
	}
}
*/

// Tests redundancy of edge
bool Model::isRedundant(ModelData::Edge* e1, int k){
	ModelData::Edge* e2;
	TM_Vector3 vs, ve;
	
	for(int i=0; i<k; i++){
		e2 = &modeldata.m_edgelist[i];
		
		// Get Vector between start-start and end-end points of edges
		vs = TM_Vector3(	modeldata.m_vertexlist[e1->start].x - modeldata.m_vertexlist[e2->start].x,
						modeldata.m_vertexlist[e1->start].y - modeldata.m_vertexlist[e2->start].y,
						modeldata.m_vertexlist[e1->start].z - modeldata.m_vertexlist[e2->start].z);
		ve = TM_Vector3(	modeldata.m_vertexlist[e1->end].x - modeldata.m_vertexlist[e2->end].x,
						modeldata.m_vertexlist[e1->end].y - modeldata.m_vertexlist[e2->end].y,
						modeldata.m_vertexlist[e1->end].z - modeldata.m_vertexlist[e2->end].z);
		// if sum of length between vertices is insignificant then redundancy is detected
		if(vs.length() + ve.length() < 0.01){
			//printf("Redundant edge detected: %d %d\n", e1->start, e1->end);
			return true;
		}
		
		// Get Vector between start-end and end-start points of edges
		vs = TM_Vector3(	modeldata.m_vertexlist[e1->start].x - modeldata.m_vertexlist[e2->end].x,
						modeldata.m_vertexlist[e1->start].y - modeldata.m_vertexlist[e2->end].y,
						modeldata.m_vertexlist[e1->start].z - modeldata.m_vertexlist[e2->end].z);
		ve = TM_Vector3(	modeldata.m_vertexlist[e1->end].x - modeldata.m_vertexlist[e2->start].x,
						modeldata.m_vertexlist[e1->end].y - modeldata.m_vertexlist[e2->start].y,
						modeldata.m_vertexlist[e1->end].z - modeldata.m_vertexlist[e2->start].z);
		// if sum of length between vertices is insignificant then redundancy is detected
		if(vs.length() + ve.length() < 0.01){
			//printf("Redundant edge detected: %d %d\n", e1->start, e1->end);
			return true;
		}
						
	}
	return false;
}

// Tests if property of file is available in list (data structure)
bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index){
	
	for(int i=0; i<n; i++){
		if(equal_strings(prop->name, list[i].name)){
			*index = i;
			return true;
		}
	}
	return false;
}

// read ply file
bool Model::read(const char* filename){
    int i,j,k,l;
    int nelems;
    char **elist;
    int file_type;
    float version;
    char *elem_name;
    PlyElement *elem_ptr;
    PlyProperty **plist;
    PlyProperty* prop_ptr;
    int num_elems;
    int nprops;
    int index;
    
    m_plyfile = ply_open_for_reading((char*)filename, &nelems, &elist, &file_type, &version);
    if(m_plyfile==0){
    	printf("[Model::read] Error loading ply file %s\n", filename);
    	return false;
    }
    
    // Load texture files from obj_info (=texture-filename)
    char** obj_info;
    int num_obj_info;   
    int id; 
    obj_info = ply_get_obj_info(m_plyfile, &num_obj_info);
   	if(num_obj_info < 1){
   		printf("[Model::read] Warning no texture found in model %s\n", filename);
   	}else if(num_obj_info >= 1){
   		id = g_Resources->AddTexture(obj_info[0]);
   		modeldata.m_tex_original = g_Resources->GetTexture(id);
   		modeldata.m_texture = modeldata.m_tex_original;
   	}
    if(num_obj_info > 1)
    	printf("[Model::read] Warning only one texture per model supported\n");

    // list of property information for a vertex
    PlyProperty vert_props[] = { 
        {(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,x), 0, 0, 0, 0},
        {(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,y), 0, 0, 0, 0},
        {(char*)"z", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,z), 0, 0, 0, 0},
        {(char*)"nx", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,nx), 0, 0, 0, 0},
        {(char*)"ny", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,ny), 0, 0, 0, 0},
        {(char*)"nz", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,nz), 0, 0, 0, 0},
        {(char*)"s", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,s), 0, 0, 0, 0},
        {(char*)"t", PLY_FLOAT, PLY_FLOAT, offsetof(ModelData::Vertex,t), 0, 0, 0, 0},
        {(char*)"red", PLY_UCHAR, PLY_UCHAR, offsetof(ModelData::Vertex,r), 0, 0, 0, 0},
        {(char*)"green", PLY_UCHAR, PLY_UCHAR, offsetof(ModelData::Vertex,g), 0, 0, 0, 0},
        {(char*)"blue", PLY_UCHAR, PLY_UCHAR, offsetof(ModelData::Vertex,b), 0, 0, 0, 0}
    };

    // list of property information for a face
    PlyProperty face_props[] = { /* list of property information for a vertex */
      {(char*)"vertex_indices", PLY_USHORT, PLY_UINT, offsetof(ModelData::Face,v),
       1, PLY_USHORT, PLY_UINT, offsetof(ModelData::Face,nverts)},
    };
    
    // list of property information for an edge
    PlyProperty edge_props[] = { 
        {(char*)"start", PLY_USHORT, PLY_USHORT, offsetof(ModelData::Edge,start), 0, 0, 0, 0},
        {(char*)"end", PLY_USHORT, PLY_USHORT, offsetof(ModelData::Edge,end), 0, 0, 0, 0}
    };
    
    for(i=0; i<nelems; i++){
        // get description of element
        elem_name = elist[i];
        plist = ply_get_element_description (m_plyfile, elem_name, &num_elems, &nprops);
        
        // *** Read Vertices ***
        if (equal_strings ((char*)"vertex", elem_name)) {
            
            // allocate memory for vertices
            modeldata.num_vertices = num_elems;
            modeldata.m_vertexlist = (ModelData::Vertex*)malloc(sizeof(ModelData::Vertex) * modeldata.num_vertices);
            
            // setup property specification for elements
            for(j=0; j<nprops; j++){
                if(propertyIsInList(plist[j], vert_props, 11, &index))
                	ply_get_property(m_plyfile, elem_name, &vert_props[index]);
            }
            
            // grab all vertex elements
            for(j=0; j<num_elems; j++){
                ply_get_element(m_plyfile, &modeldata.m_vertexlist[j]);
                
                //printf("nprops: %i vertex: %g %g %g %g %g %g %g %g %d %d %d\n", nprops,
				//		modeldata.m_vertexlist[j].x, modeldata.m_vertexlist[j].y, modeldata.m_vertexlist[j].z,
				//		modeldata.m_vertexlist[j].nx, modeldata.m_vertexlist[j].ny, modeldata.m_vertexlist[j].nz,
				//		modeldata.m_vertexlist[j].s, modeldata.m_vertexlist[j].t,
				//		modeldata.m_vertexlist[j].r, modeldata.m_vertexlist[j].g, modeldata.m_vertexlist[j].b );   
            }
            
        }
        
        // *** Read Faces ***
        if (equal_strings ((char*)"face", elem_name)) {
            // allocate memory for faces
            modeldata.num_faces = num_elems;
            modeldata.m_facelist = (ModelData::Face*)malloc(sizeof(ModelData::Face)*modeldata.num_faces);
                        
            // setup property specification for elements
            for(j=0; j<nprops && j<1; j++){
                ply_get_property(m_plyfile, elem_name, &face_props[j]);
            }
            
            // grab all face elements
            for(j=0; j<num_elems; j++){
                ply_get_element(m_plyfile, &modeldata.m_facelist[j]);
                //printf ("face: %d %d %d %d\n", modeldata.m_facelist[j].v[0], modeldata.m_facelist[j].v[1], modeldata.m_facelist[j].v[2], modeldata.m_facelist[j].v[3]);
            }            
        }
        
        // *** Read Edges ***
        if (equal_strings ((char*)"edge", elem_name)) {
            // allocate memory for edges
            modeldata.m_edgelist = (ModelData::Edge*)malloc(sizeof(ModelData::Edge)*num_elems);
            modeldata.num_edges = num_elems;
            
            // setup property specification for elements
            for(j=0; j<nprops && j<2; j++){
                ply_get_property(m_plyfile, elem_name, &edge_props[j]);
            }
            
            // grab all edge elements
            for(j=0; j<num_elems; j++){
                ply_get_element(m_plyfile, &modeldata.m_edgelist[j]);
                //printf("edge: %d %d\n", modeldata.m_edgelist[j].start, modeldata.m_edgelist[j].end);
            }
        }
    }    
    return true;
}


// *** PUBLIC ***

Model::Model(){
    m_plyfile = 0;
    loaded = false;
}

Model::~Model(){
		
	if(m_plyfile)
    	ply_close(m_plyfile);
    
    glDeleteLists(faceDisplayList,1);
    glDeleteLists(edgeDisplayList,1);

}

// initialisation of model class
bool Model::load(const char* filename){
    
    if(!loaded){
		// Load ply file
		if(!read(filename)){
			return false;
		}
		
		// Generate display lists for face and edge representation
		if(modeldata.m_facelist){
			
			genFaceDisplayList();
			
			if(!modeldata.m_edgelist)
				computeEdges();
			
			genEdgeDisplayList();
		}
		
		// Calculate normal of each vertex from faces
		computeNormals();
		
		loaded = true;
    }else{
    	printf("[Model::load] Error Model allready loaded: '%s'\n", filename);
    	return false;
    }
    
    
    return true;
}


bool Model::load(ModelData md){
	if(!loaded){
		modeldata = md;
	}else{
    	printf("[Model::load] Error Model allready loaded\n");
    	return false;
    }
}

// draws only faces of model
void Model::drawFaces(){
	int i,j;
	ModelData::Face* f;
	
	if(modeldata.m_texture){
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_2D);
		modeldata.m_texture->bind();
	}
	for(i=0; i<modeldata.num_faces; i++){
		f = &modeldata.m_facelist[i];
		glBegin(GL_QUADS);
			for(j=0; j<f->nverts; j++){
				glTexCoord2f(modeldata.m_vertexlist[f->v[j]].s, modeldata.m_vertexlist[f->v[j]].t);
				glVertex3f(modeldata.m_vertexlist[f->v[j]].x, modeldata.m_vertexlist[f->v[j]].y, modeldata.m_vertexlist[f->v[j]].z);
			}
		glEnd();
		
		/*
		glDisable(GL_TEXTURE_2D);
		glBegin(GL_LINES);
			float normal_length = 0.01;
			for(j=0; j<f->nverts; j++){
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f( modeldata.m_vertexlist[f->v[j]].x,
							modeldata.m_vertexlist[f->v[j]].y,
							modeldata.m_vertexlist[f->v[j]].z );
				glVertex3f( modeldata.m_vertexlist[f->v[j]].x + modeldata.m_vertexlist[f->v[j]].nx * normal_length,
							modeldata.m_vertexlist[f->v[j]].y + modeldata.m_vertexlist[f->v[j]].ny * normal_length,
							modeldata.m_vertexlist[f->v[j]].z + modeldata.m_vertexlist[f->v[j]].nz * normal_length );				
			}
		glEnd();
		glEnable(GL_TEXTURE_2D);
		*/
	}
	
	
	
	if(modeldata.m_texture){
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
    glPopMatrix();
}

// calls display list for faces
void Model::callFaceList(){
    if(glIsList(faceDisplayList))
    	glCallList(faceDisplayList);
}


void Model::restoreTexture(){
	modeldata.m_texture = modeldata.m_tex_original;
}

// releas resources
void Model::release(){
		
}
