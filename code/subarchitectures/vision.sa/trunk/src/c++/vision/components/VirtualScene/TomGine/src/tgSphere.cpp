
#include "tgSphere.h"

// void init_tetrahedron(){ 
//   float sqrt3 = 1 / sqrt(3.0);
//   float tetrahedron_vertices[] = {sqrt3, sqrt3, sqrt3,
// 				  -sqrt3, -sqrt3, sqrt3,
// 				  -sqrt3, sqrt3, -sqrt3,
// 				  sqrt3, -sqrt3, -sqrt3}; 
//   int tetrahedron_faces[] = {0, 2, 1, 0, 1, 3, 2, 3, 1, 3, 2, 0};
// 
//   n_vertices = 4; 
//   n_faces = 4; 
//   n_edges = 6; 
//   vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
//   faces = (int*)malloc(3*n_faces*sizeof(int)); 
//   memcpy ((void*)vertices, (void*)tetrahedron_vertices, 3*n_vertices*sizeof(float)); 
//   memcpy ((void*)faces, (void*)tetrahedron_faces, 3*n_faces*sizeof(int)); 
// } 
// 
// void init_octahedron(){ 
//   float octahedron_vertices[] = {0.0, 0.0, -1.0,
// 				 1.0, 0.0, 0.0,
// 				 0.0, -1.0, 0.0,
// 				 -1.0, 0.0, 0.0,
// 				 0.0, 1.0, 0.0,
// 				 0.0, 0.0, 1.0}; 
//   int octahedron_faces[] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1, 5, 2, 1, 5, 3, 2, 5, 4, 3, 5, 1, 4}; 
// 
//   n_vertices = 6; 
//   n_faces = 8;
//   n_edges = 12; 
//   vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
//   faces = (int*)malloc(3*n_faces*sizeof(int)); 
//   memcpy ((void*)vertices, (void*)octahedron_vertices, 3*n_vertices*sizeof(float)); 
//   memcpy ((void*)faces, (void*)octahedron_faces, 3*n_faces*sizeof(int)); 
// } 
// 
// void init_icosahedron(){ 
//   float t = (1+sqrt(5))/2;
//   float tau = t/sqrt(1+t*t);
//   float one = 1/sqrt(1+t*t);
// 
//   float icosahedron_vertices[] = {tau, one, 0.0,
// 				  -tau, one, 0.0,
// 				  -tau, -one, 0.0,
// 				  tau, -one, 0.0,
// 				  one, 0.0 ,  tau,
// 				  one, 0.0 , -tau,
// 				  -one, 0.0 , -tau,
// 				  -one, 0.0 , tau,
// 				  0.0 , tau, one,
// 				  0.0 , -tau, one,
// 				  0.0 , -tau, -one,
// 				  0.0 , tau, -one};
//  int icosahedron_faces[] = {4, 8, 7,
// 			    4, 7, 9,
// 			    5, 6, 11,
// 			    5, 10, 6,
// 			    0, 4, 3,
// 			    0, 3, 5,
// 			    2, 7, 1,
// 			    2, 1, 6,
// 			    8, 0, 11,
// 			    8, 11, 1,
// 			    9, 10, 3,
// 			    9, 2, 10,
// 			    8, 4, 0,
// 			    11, 0, 5,
// 			    4, 9, 3,
// 			    5, 3, 10,
// 			    7, 8, 1,
// 			    6, 1, 11,
// 			    7, 2, 9,
// 			    6, 10, 2};
//  
//   n_vertices = 12; 
//   n_faces = 20;
//   n_edges = 30;
//   vertices = (float*)malloc(3*n_vertices*sizeof(float)); 
//   faces = (int*)malloc(3*n_faces*sizeof(int)); 
//   memcpy ((void*)vertices, (void*)icosahedron_vertices, 3*n_vertices*sizeof(float)); 
//   memcpy ((void*)faces, (void*)icosahedron_faces, 3*n_faces*sizeof(int)); 
// } 
// 
// int search_midpoint(int index_start, int index_end){ 
//   int i;
//   for (i=0; i<edge_walk; i++) 
//     if ((start[i] == index_start && end[i] == index_end) || 
// 	(start[i] == index_end && end[i] == index_start)) 
//       {
// 	int res = midpoint[i];
// 
// 	/* update the arrays */
// 	start[i]    = start[edge_walk-1];
// 	end[i]      = end[edge_walk-1];
// 	midpoint[i] = midpoint[edge_walk-1];
// 	edge_walk--;
// 	
// 	return res; 
//       }
// 
//   /* vertex not in the list, so we add it */
//   start[edge_walk] = index_start;
//   end[edge_walk] = index_end; 
//   midpoint[edge_walk] = n_vertices; 
//   
//   /* create new vertex */ 
//   vertices[3*n_vertices]   = (vertices[3*index_start] + vertices[3*index_end]) / 2.0;
//   vertices[3*n_vertices+1] = (vertices[3*index_start+1] + vertices[3*index_end+1]) / 2.0;
//   vertices[3*n_vertices+2] = (vertices[3*index_start+2] + vertices[3*index_end+2]) / 2.0;
//   
//   /* normalize the new vertex */ 
//   float length = sqrt (vertices[3*n_vertices] * vertices[3*n_vertices] +
// 		       vertices[3*n_vertices+1] * vertices[3*n_vertices+1] +
// 		       vertices[3*n_vertices+2] * vertices[3*n_vertices+2]);
//   length = 1/length;
//   vertices[3*n_vertices] *= length;
//   vertices[3*n_vertices+1] *= length;
//   vertices[3*n_vertices+2] *= length;
//   
//   n_vertices++;
//   edge_walk++;
//   return midpoint[edge_walk-1];
// } 
// 
// void subdivide(){ 
//   int n_vertices_new = n_vertices+2*n_edges; 
//   int n_faces_new = 4*n_faces; 
//   int i; 
// 
//   edge_walk = 0; 
//   n_edges = 2*n_vertices + 3*n_faces; 
//   start = (int*)malloc(n_edges*sizeof (int)); 
//   end = (int*)malloc(n_edges*sizeof (int)); 
//   midpoint = (int*)malloc(n_edges*sizeof (int)); 
// 
//   int *faces_old = (int*)malloc (3*n_faces*sizeof(int)); 
//   faces_old = (int*)memcpy((void*)faces_old, (void*)faces, 3*n_faces*sizeof(int)); 
//   vertices = (float*)realloc ((void*)vertices, 3*n_vertices_new*sizeof(float)); 
//   faces = (int*)realloc ((void*)faces, 3*n_faces_new*sizeof(int)); 
//   n_faces_new = 0; 
// 
//   for (i=0; i<n_faces; i++) 
//     { 
//       int a = faces_old[3*i]; 
//       int b = faces_old[3*i+1]; 
//       int c = faces_old[3*i+2]; 
// 
//       int ab_midpoint = search_midpoint (b, a); 
//       int bc_midpoint = search_midpoint (c, b); 
//       int ca_midpoint = search_midpoint (a, c); 
// 
//       faces[3*n_faces_new] = a; 
//       faces[3*n_faces_new+1] = ab_midpoint; 
//       faces[3*n_faces_new+2] = ca_midpoint; 
//       n_faces_new++; 
//       faces[3*n_faces_new] = ca_midpoint; 
//       faces[3*n_faces_new+1] = ab_midpoint; 
//       faces[3*n_faces_new+2] = bc_midpoint; 
//       n_faces_new++; 
//       faces[3*n_faces_new] = ca_midpoint; 
//       faces[3*n_faces_new+1] = bc_midpoint; 
//       faces[3*n_faces_new+2] = c; 
//       n_faces_new++; 
//       faces[3*n_faces_new] = ab_midpoint; 
//       faces[3*n_faces_new+1] = b; 
//       faces[3*n_faces_new+2] = bc_midpoint; 
//       n_faces_new++; 
//     } 
//   n_faces = n_faces_new; 
//   free (start); 
//   free (end); 
//   free (midpoint); 
//   free (faces_old); 
// } 
// 
// int main (int argc, char *argv[]){ 
//   int i;
// 
//   if (!strcmp(argv[1], "-t"))
//     init_tetrahedron ();
//   if (!strcmp(argv[1], "-o"))
//     init_octahedron ();
//   if (!strcmp(argv[1], "-i"))
//     init_icosahedron ();
// 
//   int n_subdivisions = atoi (argv[2]);
//   for (i=0; i<n_subdivisions; i++) 
//     subdivide (); 
// 
// 
//   if (vertices) free (vertices); 
//   if (faces) free (faces); 
// 
//   return 0; 
// } 
// 
