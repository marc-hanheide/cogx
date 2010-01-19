
#include "ModelLoader.h"

// *** PRIVATE ***

// Tests if property of file is available in list (data structure)
bool ModelLoader::propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index){
	
	for(int i=0; i<n; i++){
		if(equal_strings(prop->name, list[i].name)){
			*index = i;
			return true;
		}
	}
	return false;
}

// *** PUBLIC ***

ModelLoader::ModelLoader(){
}

ModelLoader::~ModelLoader(){

}

// read ply file
bool ModelLoader::LoadPly(Model &model, const char* filename){
	PlyFile* plyfile;
	int i,j;
	int nelems;
	char **elist;
	int file_type;
	float version;
	char *elem_name;
	//PlyElement *elem_ptr;
	PlyProperty **plist;
	//PlyProperty* prop_ptr;
	int num_elems;
	int nprops;
	int index;
	char** obj_info;
	int num_obj_info;   
	int id; 
	
	int num_vertices = 0;
	int num_faces = 0;
	int num_edges = 0;
	PlyVertex* plyvertexlist = 0;
	PlyFace* plyfacelist = 0;
	PlyEdge* plyedgelist = 0;
	
	
	// **********************************************
	// Read data from ply file
	
	// open file
	plyfile = ply_open_for_reading((char*)filename, &nelems, &elist, &file_type, &version);
	if(plyfile==0){
		printf("[ModelLoader::read] Error loading ply file %s\n", filename);
		return false;
	}
// 	sprintf(model.m_modelname, "%s", filename);
	
	// Load texture files from obj_info (=texture-filename)
	obj_info = ply_get_obj_info(plyfile, &num_obj_info);
	if(num_obj_info < 1){
   	//printf("[ModelLoader::read] Warning no texture found in model %s\n", filename);
	}else if(num_obj_info >= 1){
		printf("[ModelLoader::read] obj_info: texture not supported\n");
	}
	
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
		plist = ply_get_element_description (plyfile, elem_name, &num_elems, &nprops);
			
		// *** Read Vertices ***
		if (equal_strings ((char*)"vertex", elem_name)) {
			// allocate memory for vertices
			num_vertices = num_elems;
			plyvertexlist = (PlyVertex*)malloc(sizeof(PlyVertex) * num_vertices);
					
			// setup property specification for elements
			for(j=0; j<nprops; j++){
				if(propertyIsInList(plist[j], vert_props, 11, &index))
				ply_get_property(plyfile, elem_name, &vert_props[index]);
			}
							
			// grab all vertex elements
			for(j=0; j<num_elems; j++){
				ply_get_element(plyfile, &plyvertexlist[j]); 
			}
				
		}
				
		// *** Read Faces ***
		if (equal_strings ((char*)"face", elem_name)) {
		// allocate memory for faces
			num_faces = num_elems;
			plyfacelist = (PlyFace*)malloc(sizeof(PlyFace)*num_faces);
									
			// setup property specification for elements
			for(j=0; j<nprops && j<1; j++){
				ply_get_property(plyfile, elem_name, &face_props[j]);
			}
			
			// grab all face elements
			for(j=0; j<num_elems; j++){
				ply_get_element(plyfile, &plyfacelist[j]);
				//printf ("face: %d %d %d %d\n", m_facelist[j].v[0], m_facelist[j].v[1], m_facelist[j].v[2], m_facelist[j].v[3]);
			}            
		}
				
		// *** Read Edges ***
		if (equal_strings ((char*)"edge", elem_name)) {
			// allocate memory for edges
			plyedgelist = (PlyEdge*)malloc(sizeof(PlyEdge)*num_elems);
			num_edges = num_elems;
			
			// setup property specification for elements
			for(j=0; j<nprops && j<2; j++){
					ply_get_property(plyfile, elem_name, &edge_props[j]);
			}
			
			// grab all edge elements
			for(j=0; j<num_elems; j++){
					ply_get_element(plyfile, &plyedgelist[j]);
					//printf("edge: %d %d\n", m_edgelist[j].start, m_edgelist[j].end);
			}
		}
	}
	ply_close(plyfile);
	
	
	// **********************************************
	// Convert ply to model	
	
	// Parse through vertex list
	for(i=0; i<num_vertices; i++){
		Model::Vertex v;
		v.pos.x = plyvertexlist[i].x;
		v.pos.y = plyvertexlist[i].y;
		v.pos.z = plyvertexlist[i].z;
		v.normal.x = plyvertexlist[i].nx;
		v.normal.y = plyvertexlist[i].ny;
		v.normal.z = plyvertexlist[i].nz;
		v.texCoord.x = plyvertexlist[i].s;
		v.texCoord.y = plyvertexlist[i].t;
		model.m_vertexlist.push_back(v);
	}
	
	// Parse through face list
	for(i=0; i<num_faces; i++){
		Model::Face f;
		for(j=0; j<plyfacelist[i].nverts; j++){
			f.v.push_back(plyfacelist[i].v[j]);
		}
		model.m_facelist.push_back(f);
	}
	
// 	for(i=0; i<num_edges; i++){
// 		Model::Edge e;
// 		e.start = plyedgelist[i].start;
// 		e.end = plyedgelist[i].end;
// 		model.m_edgelist.push_back(e);
// 	}
	
	model.computeFaceNormals();
	
	// **********************************************
	// Clean up
	if(plyvertexlist) free(plyvertexlist);
	if(plyfacelist) free(plyfacelist);
	if(plyedgelist) free(plyedgelist);
	
	return true;
}

