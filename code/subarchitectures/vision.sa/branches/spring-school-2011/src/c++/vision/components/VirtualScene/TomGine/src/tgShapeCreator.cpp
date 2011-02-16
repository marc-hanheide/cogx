
#include "tgShapeCreator.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

using namespace TomGine;



void tgShapeCreator::init_tetrahedron(){ 
  float sqrt3 = 1 / sqrt(3.0);
  float tetrahedron_vertices[] = {sqrt3, sqrt3, sqrt3,
				  -sqrt3, -sqrt3, sqrt3,
				  -sqrt3, sqrt3, -sqrt3,
				  sqrt3, -sqrt3, -sqrt3}; 
  int tetrahedron_faces[] = {0, 2, 1, 0, 1, 3, 2, 3, 1, 3, 2, 0};

  n_vertices = 4; 
  n_faces = 4; 
  n_edges = 6; 
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (int*)malloc(3*n_faces*sizeof(int)); 
  memcpy ((void*)vertices, (void*)tetrahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)tetrahedron_faces, 3*n_faces*sizeof(int)); 
} 

void tgShapeCreator::init_octahedron(){ 
  float octahedron_vertices[] = {0.0, 0.0, -1.0,
				 1.0, 0.0, 0.0,
				 0.0, -1.0, 0.0,
				 -1.0, 0.0, 0.0,
				 0.0, 1.0, 0.0,
				 0.0, 0.0, 1.0}; 
  int octahedron_faces[] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 5, 2, 1, 5, 3, 2, 5, 4, 3, 5, 1, 4}; 

  n_vertices = 6; 
  n_faces = 8;
  n_edges = 12; 
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (int*)malloc(3*n_faces*sizeof(int)); 
  memcpy ((void*)vertices, (void*)octahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)octahedron_faces, 3*n_faces*sizeof(int)); 
} 

void tgShapeCreator::init_icosahedron(){ 
  float t = (1+sqrt(5))/2;
  float tau = t/sqrt(1+t*t);
  float one = 1/sqrt(1+t*t);

  float icosahedron_vertices[] = {tau, one, 0.0,
				  -tau, one, 0.0,
				  -tau, -one, 0.0,
				  tau, -one, 0.0,
				  one, 0.0 ,  tau,
				  one, 0.0 , -tau,
				  -one, 0.0 , -tau,
				  -one, 0.0 , tau,
				  0.0 , tau, one,
				  0.0 , -tau, one,
				  0.0 , -tau, -one,
				  0.0 , tau, -one};
 int icosahedron_faces[] = {4, 8, 7,
			    4, 7, 9,
			    5, 6, 11,
			    5, 10, 6,
			    0, 4, 3,
			    0, 3, 5,
			    2, 7, 1,
			    2, 1, 6,
			    8, 0, 11,
			    8, 11, 1,
			    9, 10, 3,
			    9, 2, 10,
			    8, 4, 0,
			    11, 0, 5,
			    4, 9, 3,
			    5, 3, 10,
			    7, 8, 1,
			    6, 1, 11,
			    7, 2, 9,
			    6, 10, 2};
 
  n_vertices = 12; 
  n_faces = 20;
  n_edges = 30;
  vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
  faces = (int*)malloc(3*n_faces*sizeof(int)); 
  memcpy ((void*)vertices, (void*)icosahedron_vertices, 3*n_vertices*sizeof(float)); 
  memcpy ((void*)faces, (void*)icosahedron_faces, 3*n_faces*sizeof(int)); 
} 

int tgShapeCreator::search_midpoint(int index_start, int index_end){ 
  int i;
  for (i=0; i<edge_walk; i++) 
    if ((start[i] == index_start && end[i] == index_end) || 
	(start[i] == index_end && end[i] == index_start)) 
      {
	int res = midpoint[i];

	/* update the arrays */
	start[i]    = start[edge_walk-1];
	end[i]      = end[edge_walk-1];
	midpoint[i] = midpoint[edge_walk-1];
	edge_walk--;
	
	return res; 
      }

  /* vertex not in the list, so we add it */
  start[edge_walk] = index_start;
  end[edge_walk] = index_end; 
  midpoint[edge_walk] = n_vertices; 
  
  /* create new vertex */ 
  vertices[3*n_vertices]   = (vertices[3*index_start] + vertices[3*index_end]) / 2.0;
  vertices[3*n_vertices+1] = (vertices[3*index_start+1] + vertices[3*index_end+1]) / 2.0;
  vertices[3*n_vertices+2] = (vertices[3*index_start+2] + vertices[3*index_end+2]) / 2.0;
  
  /* normalize the new vertex */ 
  float length = sqrt (vertices[3*n_vertices] * vertices[3*n_vertices] +
		       vertices[3*n_vertices+1] * vertices[3*n_vertices+1] +
		       vertices[3*n_vertices+2] * vertices[3*n_vertices+2]);
  length = 1/length;
  vertices[3*n_vertices] *= length;
  vertices[3*n_vertices+1] *= length;
  vertices[3*n_vertices+2] *= length;
  
  n_vertices++;
  edge_walk++;
  return midpoint[edge_walk-1];
} 

void tgShapeCreator::subdivide(){ 
  int n_vertices_new = n_vertices+2*n_edges; 
  int n_faces_new = 4*n_faces; 
  int i; 

  edge_walk = 0; 
  n_edges = 2*n_vertices + 3*n_faces; 
  start = (int*)malloc(n_edges*sizeof (int)); 
  end = (int*)malloc(n_edges*sizeof (int)); 
  midpoint = (int*)malloc(n_edges*sizeof (int)); 

  int *faces_old = (int*)malloc (3*n_faces*sizeof(int)); 
  faces_old = (int*)memcpy((void*)faces_old, (void*)faces, 3*n_faces*sizeof(int)); 
  vertices = (float*)realloc ((void*)vertices, 3*n_vertices_new*sizeof(float)); 
  faces = (int*)realloc ((void*)faces, 3*n_faces_new*sizeof(int)); 
  n_faces_new = 0; 

  for (i=0; i<n_faces; i++) 
    { 
      int a = faces_old[3*i]; 
      int b = faces_old[3*i+1]; 
      int c = faces_old[3*i+2]; 

      int ab_midpoint = search_midpoint (b, a); 
      int bc_midpoint = search_midpoint (c, b); 
      int ca_midpoint = search_midpoint (a, c); 

      faces[3*n_faces_new] = a; 
      faces[3*n_faces_new+1] = ab_midpoint; 
      faces[3*n_faces_new+2] = ca_midpoint; 
      n_faces_new++; 
      faces[3*n_faces_new] = ca_midpoint; 
      faces[3*n_faces_new+1] = ab_midpoint; 
      faces[3*n_faces_new+2] = bc_midpoint; 
      n_faces_new++; 
      faces[3*n_faces_new] = ca_midpoint; 
      faces[3*n_faces_new+1] = bc_midpoint; 
      faces[3*n_faces_new+2] = c; 
      n_faces_new++; 
      faces[3*n_faces_new] = ab_midpoint; 
      faces[3*n_faces_new+1] = b; 
      faces[3*n_faces_new+2] = bc_midpoint; 
      n_faces_new++; 
    } 
  n_faces = n_faces_new; 
  free (start); 
  free (end); 
  free (midpoint); 
  free (faces_old); 
} 

tgShapeCreator::tgShapeCreator(){
	vertices = 0;
	faces = 0; 
	start = 0; 
	end = 0; 
	midpoint = 0; 
}

void tgShapeCreator::CreateSphere(tgModel& model, float radius, int subdevisions, int method){
	int i;
	int vidx = model.GetVerticesSize();
	tgModel::Vertex v;
	tgModel::Face f;
	
	// Basic geometry used for sphere
	switch(method){
		case TETRAHEDRON:
			init_tetrahedron ();
			break;
		case OCTAHEDRON:
			init_octahedron ();
			break;
		case ICOSAHEDRON:
			init_icosahedron();
			break;
		default:
			init_icosahedron();
			break;
	}

	// Subdevide basic geometry
  for(i=0; i<subdevisions; i++) 
    subdivide();
   
  // Copy vertices
  for(i=0; i<n_vertices; i++){
  	v.pos.x = radius * vertices[3*i+0];
  	v.pos.y = radius * vertices[3*i+1];
  	v.pos.z = radius * vertices[3*i+2];
  	v.normal = v.pos;
  	v.normal.normalize();
  	model.m_vertices.push_back(v);
  }
 
  // Copy faces
  for(i=0; i<n_faces; i++){
  	f.vertices.clear();
		f.vertices.push_back(vidx+faces[3*i+0]);
		f.vertices.push_back(vidx+faces[3*i+1]);
		f.vertices.push_back(vidx+faces[3*i+2]);
		model.m_faces.push_back(f);
	}

  if(vertices) free (vertices); 
  if(faces) free (faces); 

} 

void tgShapeCreator::CreateBox(tgModel& model, float x, float y, float z){
	tgModel::Vertex v;
	tgModel::Face f;
	int vidx = model.GetVerticesSize();
	x = x*0.5;
	y = y*0.5;
	z = z*0.5;
	
	
	// Front
	v.pos = vec3(-x,-y, z); v.normal = vec3( 0.0, 0.0, 1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x,-y, z); v.normal = vec3( 0.0, 0.0, 1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x, y, z); v.normal = vec3( 0.0, 0.0, 1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x, y, z); v.normal = vec3( 0.0, 0.0, 1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	model.m_faces.push_back(f); f.vertices.clear();
	
	// Back
	v.pos = vec3( x,-y,-z); v.normal = vec3( 0.0, 0.0,-1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x,-y,-z); v.normal = vec3( 0.0, 0.0,-1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x, y,-z); v.normal = vec3( 0.0, 0.0,-1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x, y,-z); v.normal = vec3( 0.0, 0.0,-1.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	model.m_faces.push_back(f); f.vertices.clear();
	
	// Right
	v.pos = vec3( x,-y, z); v.normal = vec3( 1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x,-y,-z); v.normal = vec3( 1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x, y,-z); v.normal = vec3( 1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x, y, z); v.normal = vec3( 1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	model.m_faces.push_back(f); f.vertices.clear();
	
	// Left
	v.pos = vec3(-x,-y,-z); v.normal = vec3(-1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x,-y, z); v.normal = vec3(-1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x, y, z); v.normal = vec3(-1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x, y,-z); v.normal = vec3(-1.0, 0.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	model.m_faces.push_back(f); f.vertices.clear();
	
	// Top
	v.pos = vec3(-x, y, z); v.normal = vec3( 0.0, 1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x, y, z); v.normal = vec3( 0.0, 1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x, y,-z); v.normal = vec3( 0.0, 1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x, y,-z); v.normal = vec3( 0.0, 1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	model.m_faces.push_back(f); f.vertices.clear();
	
	// Bottom
	v.pos = vec3( x,-y, z); v.normal = vec3( 0.0,-1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x,-y, z); v.normal = vec3( 0.0,-1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3(-x,-y,-z); v.normal = vec3( 0.0,-1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	v.pos = vec3( x,-y,-z); v.normal = vec3( 0.0,-1.0, 0.0); model.m_vertices.push_back(v); f.vertices.push_back(vidx++);
	model.m_faces.push_back(f); f.vertices.clear();
}

void tgShapeCreator::CreateCylinder(tgModel &model, float radius, float height, int slices, int stacks, bool closed){
	int x,z;
	int vidx = model.GetVerticesSize();
	float alpha = 2*PI / slices;
	float deltaH = height / stacks;
	tgModel::Vertex v[4];
	tgModel::Face f;
	
	for(z=0; z<stacks; z++){
		for(x=0; x<slices; x++){
			f.vertices.clear();
			
			v[0].pos.x = radius * cos(alpha*x);
			v[0].pos.y = radius * sin(alpha*x);
			v[0].pos.z = deltaH*z - height*0.5;
			v[0].normal.x = v[0].pos.x;
			v[0].normal.y = v[0].pos.y;
			v[0].normal.z = 0.0;
			v[0].normal = normalize(v[0].normal);
			model.m_vertices.push_back(v[0]);
			f.vertices.push_back(vidx++);
			
			v[1].pos.x = radius * cos(alpha*(x+1));
			v[1].pos.y = radius * sin(alpha*(x+1));
			v[1].pos.z = deltaH*z - height*0.5;
			v[1].normal.x = v[1].pos.x;
			v[1].normal.y = v[1].pos.y;
			v[1].normal.z = 0.0;
			v[1].normal = normalize(v[1].normal);
			model.m_vertices.push_back(v[1]);
			f.vertices.push_back(vidx++);
			
			v[2].pos.x = radius * cos(alpha*(x+1));
			v[2].pos.y = radius * sin(alpha*(x+1));
			v[2].pos.z = deltaH*(z+1) - height*0.5;
			v[2].normal.x = v[2].pos.x;
			v[2].normal.y = v[2].pos.y;
			v[2].normal.z = 0.0;
			v[2].normal = normalize(v[2].normal);
			model.m_vertices.push_back(v[2]);
			f.vertices.push_back(vidx++);
			
			v[3].pos.x = radius * cos(alpha*x);
			v[3].pos.y = radius * sin(alpha*x);
			v[3].pos.z = deltaH*(z+1) - height*0.5;
			v[3].normal.x = v[3].pos.x;
			v[3].normal.y = v[3].pos.y;
			v[3].normal.z = 0.0;
			v[3].normal = normalize(v[3].normal);
			model.m_vertices.push_back(v[3]);
			f.vertices.push_back(vidx++);
			model.m_faces.push_back(f);
		}
	}
	
	if(closed){
		for(z=0; z<2; z++){
			deltaH = height*z - height*0.5;
			
			for(x=0; x<slices; x++){
				f.vertices.clear();
				
				v[0].pos = vec3(0.0,0.0,deltaH);
				v[0].normal = normalize(v[0].pos);
				model.m_vertices.push_back(v[0]);
				f.vertices.push_back(vidx++);
				
				v[1].pos.x = radius * cos(alpha*x);
				v[1].pos.y = radius * sin(alpha*x);
				v[1].pos.z = deltaH;
				v[1].normal = v[0].normal;
				model.m_vertices.push_back(v[1]);
				f.vertices.push_back(vidx++);
				
				v[2].pos.x = radius * cos(alpha*(x+(float(z)-0.5)*2.0));
				v[2].pos.y = radius * sin(alpha*(x+(float(z)-0.5)*2.0));
				v[2].pos.z = deltaH;
				v[2].normal = v[0].normal;
				model.m_vertices.push_back(v[2]);
				f.vertices.push_back(vidx++);
				model.m_faces.push_back(f);
			}
		}
	}
}

