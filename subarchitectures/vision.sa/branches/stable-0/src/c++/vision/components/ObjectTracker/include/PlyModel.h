
#ifndef __PLYMODEL_H__
#define __PLYMODEL_H__


#include <stdlib.h>
#include <stddef.h>
#include <ply.h>

#include "Model.h"
#include "TM_Vector3.h"
#include "mathlib.h"

class PlyModel : public Model
{
private:

	// Variables
    typedef struct PlyVertex {
        float x,y,z;                // spatial position
        float nx, ny, nz;           // normal vector
        float s, t;
        unsigned char r, g, b;      // color
	} PlyVertex;

    typedef struct PlyFace {
        unsigned short nverts;      // number of vertices used for the face (Quad=4, Triangle=3)
        unsigned int* v;            // pointer to memory holding the vertex-index list
        float t[3], b[3], n[3];		// Tangent space vectors
    } PlyFace;

    typedef struct PlyEdge {
        unsigned short start;       // start vertex index
        unsigned short end;         // end vertex index
    } PlyEdge;
    
    PlyFile* m_plyfile;
    
    PlyVertex* m_plyvertexlist;
    PlyVertex* m_plyvertexlist_original;
    PlyFace* m_plyfacelist;
    PlyEdge* m_plyedgelist;
    int num_vertices;
    int num_faces;
    int num_edges;
    
    void printply();
    bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);
    bool read(const char* filename);
    bool convertPlyModel();


public:
    PlyModel();
    ~PlyModel();
    
    //void write(const char* filename);
    bool load(const char* filename);
   
};


#endif
