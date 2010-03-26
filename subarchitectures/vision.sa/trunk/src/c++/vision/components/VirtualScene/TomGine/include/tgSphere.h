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

namespace TomGine{

class tgSphere{
private:
	int n_vertices;
	int n_faces;
	int n_edges;
	std::vector<float> vertices;
	std::vector<int> faces; 
	
	int edge_walk; 
	std::vector<int> start; 
	std::vector<int> end; 
	std::vector<int> midpoint; 
	
 	void init_tetrahedron();
 	void init_octahedron();
 	void init_icosahedron();
 	
 	int search_midpoint(int index_start, int index_end);
	void subdivide();

public:
 	
 	void CreateSphere(tgModel& model, float radius, int subdevisions);
 	
 	
};

}
 
 #endif