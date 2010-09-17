
#include "PlyModel.h"
#include "Resources.h"

// *** PRIVATE ***

void PlyModel::printply(){
	int i,j;
	
	printf("PlyModel:\n");
	for(i=0; i<num_vertices; i++){
		PlyVertex v = m_plyvertexlist[i];
		printf("PlyVertex %i: %f %f %f, %f %f %f, %f %f\n", i, v.x, v.y, v.z, v.nx, v.ny, v.nz, v.s, v.t);
	}
	for(i=0; i<num_faces; i++){
		printf("PlyFace %i: ",i);
		for(j=0; j<m_plyfacelist[i].nverts; j++){
			printf("%i ", m_plyfacelist[i].v[j]);
		}
		printf("\n");
	}
	for(i=0; i<num_edges; i++){
		printf("PlyEdge %i: %i %i\n", i, m_plyedgelist[i].start, m_plyedgelist[i].end);
	}
}

// Tests if property of file is available in list (data structure)
bool PlyModel::propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index){
	
	for(int i=0; i<n; i++){
		if(equal_strings(prop->name, list[i].name)){
			*index = i;
			return true;
		}
	}
	return false;
}

// read ply file

// read ply file
bool PlyModel::read(const char* filename){
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
    	printf("[PlyModel::read] Error loading ply file %s\n", filename);
    	return false;
    }
    sprintf(m_modelname, "%s", filename);
    
    // Load texture files from obj_info (=texture-filename)
    char** obj_info;
    int num_obj_info;   
    int id; 
    obj_info = ply_get_obj_info(m_plyfile, &num_obj_info);
   	if(num_obj_info < 1){
   		printf("[PlyModel::read] Warning no texture found in model %s\n", filename);
   	}else if(num_obj_info >= 1){
   		id = g_Resources->AddTexture(obj_info[0]);
   		m_tex_original = g_Resources->GetTexture(id);
   		m_texture = m_tex_original;
   		m_textured = true;
   	}
    if(num_obj_info > 1)
    	printf("[PlyModel::read] Warning only one texture per model supported\n");

    // list of property information for a vertex
    PlyProperty vert_props[] = { 
        {(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,x), 0, 0, 0, 0},
        {(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,y), 0, 0, 0, 0},
        {(char*)"z", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,z), 0, 0, 0, 0},
        {(char*)"nx", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nx), 0, 0, 0, 0},
        {(char*)"ny", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,ny), 0, 0, 0, 0},
        {(char*)"nz", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nz), 0, 0, 0, 0},
        {(char*)"s", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,s), 0, 0, 0, 0},
        {(char*)"t", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,t), 0, 0, 0, 0},
        {(char*)"red", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,r), 0, 0, 0, 0},
        {(char*)"green", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,g), 0, 0, 0, 0},
        {(char*)"blue", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,b), 0, 0, 0, 0}
    };

    // list of property information for a face
    PlyProperty face_props[] = { /* list of property information for a vertex */
      {(char*)"vertex_indices", PLY_USHORT, PLY_UINT, offsetof(PlyFace,v),
       1, PLY_USHORT, PLY_UINT, offsetof(PlyFace,nverts)},
    };
    
    // list of property information for an edge
    PlyProperty edge_props[] = { 
        {(char*)"start", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,start), 0, 0, 0, 0},
        {(char*)"end", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,end), 0, 0, 0, 0}
    };
    
    for(i=0; i<nelems; i++){
        // get description of element
        elem_name = elist[i];
        plist = ply_get_element_description (m_plyfile, elem_name, &num_elems, &nprops);
        
        // *** Read Vertices ***
        if (equal_strings ((char*)"vertex", elem_name)) {
            // allocate memory for vertices
            num_vertices = num_elems;
            m_plyvertexlist = (PlyVertex*)malloc(sizeof(PlyVertex) * num_vertices);
            m_plyvertexlist_original = (PlyVertex*)malloc(sizeof(PlyVertex) * num_vertices);
            
            // setup property specification for elements
            for(j=0; j<nprops; j++){
                if(propertyIsInList(plist[j], vert_props, 11, &index))
                	ply_get_property(m_plyfile, elem_name, &vert_props[index]);
            }
            
            // grab all vertex elements
            for(j=0; j<num_elems; j++){
                ply_get_element(m_plyfile, &m_plyvertexlist[j]);
                m_plyvertexlist_original[j] = m_plyvertexlist[j];
                
                //printf("nprops: %i vertex: %g %g %g %g %g %g %g %g %d %d %d\n", nprops,
				//		m_plyvertexlist[j].x, m_plyvertexlist[j].y, m_plyvertexlist[j].z,
				//		m_plyvertexlist[j].nx, m_plyvertexlist[j].ny, m_plyvertexlist[j].nz,
				//		m_plyvertexlist[j].s, m_plyvertexlist[j].t,
				//		m_plyvertexlist[j].r, m_plyvertexlist[j].g, m_plyvertexlist[j].b );   
            }
            
        }
        
        // *** Read Faces ***
        if (equal_strings ((char*)"face", elem_name)) {
        	// allocate memory for faces
            num_faces = num_elems;
            m_plyfacelist = (PlyFace*)malloc(sizeof(PlyFace)*num_faces);
                        
            // setup property specification for elements
            for(j=0; j<nprops && j<1; j++){
                ply_get_property(m_plyfile, elem_name, &face_props[j]);
            }
            
            // grab all face elements
            for(j=0; j<num_elems; j++){
                ply_get_element(m_plyfile, &m_plyfacelist[j]);
                //printf ("face: %d %d %d %d\n", m_facelist[j].v[0], m_facelist[j].v[1], m_facelist[j].v[2], m_facelist[j].v[3]);
            }            
        }
        
        // *** Read Edges ***
        if (equal_strings ((char*)"edge", elem_name)) {
        	// allocate memory for edges
            m_plyedgelist = (PlyEdge*)malloc(sizeof(PlyEdge)*num_elems);
            num_edges = num_elems;
            
            // setup property specification for elements
            for(j=0; j<nprops && j<2; j++){
                ply_get_property(m_plyfile, elem_name, &edge_props[j]);
            }
            
            // grab all edge elements
            for(j=0; j<num_elems; j++){
                ply_get_element(m_plyfile, &m_plyedgelist[j]);
                //printf("edge: %d %d\n", m_edgelist[j].start, m_edgelist[j].end);
            }
        }
    }
    
    return true;
}


bool PlyModel::convertPlyModel(){
	int i,j;
	
	// Parse through vertex list
	for(i=0; i<num_vertices; i++){
		Vertex v;
		v.pos.x = m_plyvertexlist[i].x;
		v.pos.y = m_plyvertexlist[i].y;
		v.pos.z = m_plyvertexlist[i].z;
		v.normal.x = m_plyvertexlist[i].nx;
		v.normal.y = m_plyvertexlist[i].nz;
		v.normal.z = m_plyvertexlist[i].ny;
		v.texCoord.x = m_plyvertexlist[i].s;
		v.texCoord.y = m_plyvertexlist[i].t;
		m_vertexlist.push_back(v);
	}
	
	// Parse through face list
	for(i=0; i<num_faces; i++){
		Face f;
		for(j=0; j<m_plyfacelist[i].nverts; j++){
			f.v.push_back(m_plyfacelist[i].v[j]);
		}
		f.max_pixels = 0;
		m_facelist.push_back(f);
	}
	
	for(i=0; i<num_edges; i++){
		Edge e;
		e.start = m_plyedgelist[i].start;
		e.end = m_plyedgelist[i].end;
		m_edgelist.push_back(e);
	}	
	
	return true;
}

// *** PUBLIC ***

PlyModel::PlyModel(){
	m_plyvertexlist = 0;
    m_plyfacelist = 0;
    m_plyedgelist = 0;
	num_vertices = 0;
	num_faces = 0;
	num_edges = 0;
}

PlyModel::~PlyModel(){
	if(m_plyvertexlist)
		free(m_plyvertexlist);
	if(m_plyfacelist)
		free(m_plyfacelist);
	if(m_plyedgelist)
		free(m_plyedgelist);
		
	if(m_plyfile)
    	ply_close(m_plyfile);
}

// initialisation of model class
bool PlyModel::load(const char* filename){
    
    // Load ply file
	if(!read(filename)){
    	return false;
    }
    
    convertPlyModel();
    
    // Generate display lists for face and edge representation
	if(m_edgelist.size() == 0){
		computeEdges();
	}
	
	//genEdgeDisplayList();
    
    // Calculate normal of each vertex from faces
    computeNormals();
    
    return true;
}
