/**
* @file tgSphere.h
* @author Thomas Mörwald
* @date March 2010
* @version 0.1
* @brief Generating a sphere mesh using a subdevided icosahedra.
* @namespace TomGine
*/

#ifndef TGSPHERE_H
#define TGSPHERE_H

#include <vector>

#include "tgModel.h"

#define TETRAHEDRON 0
#define OCTAHEDRON 1
#define ICOSAHEDRON 2

namespace TomGine{

class tgSphere{
private:
	int n_vertices;
	int n_faces;
	int n_edges;
	int edge_walk; 
	float *vertices;
	int *faces; 
	int *start; 
	int *end; 
	int *midpoint; 
	
 	void init_tetrahedron();
 	void init_octahedron();
 	void init_icosahedron();
 	
 	int search_midpoint(int index_start, int index_end);
	void subdivide();

public:
	tgSphere();
 	
 	void CreateSphere(tgModel& model, float radius, int subdevisions, int method=0);
 	
 	
};

}
 
 #endif