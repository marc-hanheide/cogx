
#ifndef __MODEL_H__
#define __MODEL_H__

class Model;

#include <stdlib.h>
#include <stddef.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <highgui.h>
#include <ply.h>

#include "TM_Vector3.h"
#include "mathlib.h"
#include "Resources.h"

struct ModelData{
    typedef struct Vertex {
        float x,y,z;                // spatial position
        float nx, ny, nz;           // normal vector
        float s, t;
        unsigned char r, g, b;      // color
    } Vertex;

    typedef struct Face {
        unsigned short nverts;      // number of vertices used for the face (Quad=4, Triangle=3)
        unsigned int* v;            // pointer to memory holding the vertex-index list
        //float t[3], b[3], n[3];		// Tangent space vectors
    } Face;

    typedef struct Edge {
        unsigned short start;       // start vertex index
        unsigned short end;         // end vertex index
    } Edge;

	int num_vertices;               // total number of vertices
    int num_faces;                  // total number of faces
    int num_edges;                  // total number of edges
    Vertex* m_vertexlist;           // pointer to memory holding the vertex list
    Face* m_facelist;               // pointer to memory holding the face list
    Edge* m_edgelist;               // pointer to memory holding the edge list
    Texture* m_tex_original;		// original texture of model (not modified by tracker)
    Texture* m_texture;				// texture of model modified by tracker (edge-texture)
	
    
    ModelData();
    ~ModelData();
    
    ModelData& operator=(const ModelData& md2);
	
};

class Model
{
private:
    
	PlyFile* m_plyfile;
    
    ModelData modeldata;
    bool loaded;
    
    GLint faceDisplayList;          // glDisplayList for drawing faces
    GLint edgeDisplayList;          // glDisplayList for drawing edges
    
    // Functions
    void genFaceDisplayList();
    void genEdgeDisplayList();
    //void computeTBN();
    bool isRedundant(ModelData::Edge* e1, int k);
    bool read(const char* filename);

public:
    Model();
    ~Model();
    
    void computeEdges();
    void computeNormals();
    
    void write(const char* filename);
    bool load(const char* filename);
    bool load(ModelData md);
    
    Texture* getTexture(){ return modeldata.m_texture; }
    
    void setTexture(Texture* tex){ modeldata.m_texture = tex; }
    
    void drawFaces();
    void drawEdges();
    void callFaceList();
    void callEdgeList();
    
    void restoreTexCoords();
    void restoreTexture();
    
    void release();
    
};


#endif
