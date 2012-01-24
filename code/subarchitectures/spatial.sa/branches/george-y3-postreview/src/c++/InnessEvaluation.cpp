#include <cast/core/CASTUtils.hpp>
#include "RelationEvaluation.hpp"
using namespace cast;
#include <Pose3.h>
#include <iostream>
#include <set>
  
using namespace std;
using namespace cogx::Math;


namespace spatial {


double
RelationEvaluator::evaluateInness(const Object *objectC, const Object *objectO)
{
  if (objectC->type == OBJECT_BOX ||
      objectC->type == OBJECT_PLANE ||
      objectC->type == OBJECT_SPHERE ||
      objectC->type == OBJECT_CYLINDER) {
    return 0;
  }

  // Clip objectO to the convex hull of objectC. Compute ratio
  // of clipped volume to total volume.
  // Also compute depth of penetration of objectO into objectC.

  // In-ness is the volume ratio, penalized by degree of penetration.

  double entireVolume;
  // Convert objectO into a Polyhedron
  Polyhedron polyO;
  if (objectO->type == OBJECT_PLANE) {
    PlaneObject *plane = (PlaneObject*) objectO;

    if (plane->shape == PLANE_OBJECT_RECTANGLE) {
      double radius1 = plane->radius1;
      double radius2 = plane->radius2;
      polyO.vertices.push_back(vector3(radius1,radius2,0));
      polyO.vertices.push_back(vector3(-radius1,radius2,0));
      polyO.vertices.push_back(vector3(-radius1,-radius2,0));
      polyO.vertices.push_back(vector3(radius1,-radius2,0));
      polyO.vertices.push_back(vector3(radius1,radius2,-planeThickness));
      polyO.vertices.push_back(vector3(-radius1,radius2,-planeThickness));
      polyO.vertices.push_back(vector3(-radius1,-radius2,-planeThickness));
      polyO.vertices.push_back(vector3(radius1,-radius2,-planeThickness));
      polyO.faces.resize(6);
      polyO.faces[0].push_back(Edge(0,1));
      polyO.faces[0].push_back(Edge(1,2));
      polyO.faces[0].push_back(Edge(2,3));
      polyO.faces[0].push_back(Edge(3,0));
      polyO.faces[1].push_back(Edge(4,7));
      polyO.faces[1].push_back(Edge(7,6));
      polyO.faces[1].push_back(Edge(6,5));
      polyO.faces[1].push_back(Edge(5,4));
      polyO.faces[2].push_back(Edge(0,4));
      polyO.faces[2].push_back(Edge(4,5));
      polyO.faces[2].push_back(Edge(5,1));
      polyO.faces[2].push_back(Edge(1,0));
      polyO.faces[3].push_back(Edge(1,5));
      polyO.faces[3].push_back(Edge(5,6));
      polyO.faces[3].push_back(Edge(6,2));
      polyO.faces[3].push_back(Edge(2,1));
      polyO.faces[4].push_back(Edge(2,6));
      polyO.faces[4].push_back(Edge(6,7));
      polyO.faces[4].push_back(Edge(7,3));
      polyO.faces[4].push_back(Edge(3,2));
      polyO.faces[5].push_back(Edge(3,7));
      polyO.faces[5].push_back(Edge(7,4));
      polyO.faces[5].push_back(Edge(4,0));
      polyO.faces[5].push_back(Edge(0,3));
      entireVolume = radius1*radius2*planeThickness*4;
    }
    else {
      //SHAPE_CIRCLE
      int N = 2;
      double err;
      double radius = plane->radius1;
      do {
	N++;
	err = radius * (1-cos(M_PI/N));
      } while (err > circlePlaneApproximationThreshold);

      polyO.vertices.reserve(N*2);
      polyO.faces = vector<vector<Edge> >(2+N);
      double step = 2*M_PI/N;
      for (int i = 0; i < N-1; i++) {
	polyO.vertices.push_back(vector3(radius*cos(i*step),
	      radius*sin(i*step), 0));
	polyO.faces[0].push_back(Edge(i,i+1));
	polyO.faces[1].push_back(Edge(2*N-1-i, 2*N-2-i));
	polyO.faces[1+i].push_back(Edge(i+1,i));
	polyO.faces[1+i].push_back(Edge(i,i+N));
	polyO.faces[1+i].push_back(Edge(i+N,i+N+1));
	polyO.faces[1+i].push_back(Edge(i+N+1,i+1));
      }
      polyO.faces[0].push_back(Edge(N-1,0));
      polyO.faces[1].push_back(Edge(N,2*N-1));
      polyO.faces[N+1].push_back(Edge(0,N-1));
      polyO.faces[N+1].push_back(Edge(N-1,2*N-1));
      polyO.faces[N+1].push_back(Edge(2*N-1,N));
      polyO.faces[N+1].push_back(Edge(N,0));
      for (int i = 0; i < N-1; i++) {
	polyO.vertices.push_back(vector3(radius*cos(i*step),
	      radius*sin(i*step), -planeThickness));
      }

      entireVolume = planeThickness * N*radius*cos(M_PI/N)*sin(M_PI/N);
    }
  }
  else if (objectO->type == OBJECT_BOX || objectO->type == OBJECT_HOLLOW_BOX) {
    BoxObject *box = (BoxObject *)objectO;
    double radius1 = box->radius1;
    double radius2 = box->radius2;
    double radius3 = box->radius3;
    polyO.vertices.push_back(vector3(radius1,radius2,radius3));
    polyO.vertices.push_back(vector3(-radius1,radius2,radius3));
    polyO.vertices.push_back(vector3(-radius1,-radius2,radius3));
    polyO.vertices.push_back(vector3(radius1,-radius2,radius3));
    polyO.vertices.push_back(vector3(radius1,radius2,-radius3));
    polyO.vertices.push_back(vector3(-radius1,radius2,-radius3));
    polyO.vertices.push_back(vector3(-radius1,-radius2,-radius3));
    polyO.vertices.push_back(vector3(radius1,-radius2,-radius3));
    polyO.faces.resize(6);
    polyO.faces[0].push_back(Edge(0,1));
    polyO.faces[0].push_back(Edge(1,2));
    polyO.faces[0].push_back(Edge(2,3));
    polyO.faces[0].push_back(Edge(3,0));
    polyO.faces[1].push_back(Edge(4,7));
    polyO.faces[1].push_back(Edge(7,6));
    polyO.faces[1].push_back(Edge(6,5));
    polyO.faces[1].push_back(Edge(5,4));
    polyO.faces[2].push_back(Edge(0,4));
    polyO.faces[2].push_back(Edge(4,5));
    polyO.faces[2].push_back(Edge(5,1));
    polyO.faces[2].push_back(Edge(1,0));
    polyO.faces[3].push_back(Edge(1,5));
    polyO.faces[3].push_back(Edge(5,6));
    polyO.faces[3].push_back(Edge(6,2));
    polyO.faces[3].push_back(Edge(2,1));
    polyO.faces[4].push_back(Edge(2,6));
    polyO.faces[4].push_back(Edge(6,7));
    polyO.faces[4].push_back(Edge(7,3));
    polyO.faces[4].push_back(Edge(3,2));
    polyO.faces[5].push_back(Edge(3,7));
    polyO.faces[5].push_back(Edge(7,4));
    polyO.faces[5].push_back(Edge(4,0));
    polyO.faces[5].push_back(Edge(0,3));
    entireVolume = radius1*radius2*radius3*8;
  }
  else {
    cerr << "Object type not supported yet!\n";
    exit(1);
  }

  double containedVolume;

  if (objectC->type == OBJECT_HOLLOW_BOX) {
    //Clip ObjectO's polyhedron to the convex hull of ObjectC
    BoxObject *box = (BoxObject*) objectC;

    for (unsigned int i = 0; i < polyO.vertices.size(); i++) {
      polyO.vertices[i] = transform(objectO->pose, polyO.vertices[i]);
      polyO.vertices[i] = transformInverse(objectC->pose, polyO.vertices[i]);
    }
    clipPolyhedronToPlane(polyO, vector3(box->radius1,0,0), vector3(-1,0,0));
    clipPolyhedronToPlane(polyO, vector3(-box->radius1,0,0), vector3(1,0,0));
    clipPolyhedronToPlane(polyO, vector3(0, box->radius2,0), vector3(0,-1,0));
    clipPolyhedronToPlane(polyO, vector3(0, -box->radius2,0), vector3(0,1,0));
    clipPolyhedronToPlane(polyO, vector3(0, 0, box->radius3), vector3(0,0,-1));
    clipPolyhedronToPlane(polyO, vector3(0, 0, -box->radius3), vector3(0,0,1));

    mergeAnyOverlappingVertices(polyO, 1e-6);
  try {
    containedVolume = computePolyhedronVolume(polyO);
  }
  catch(exception e) {
    cerr << ("Oh noes");
    containedVolume = 0;
  }
  }
  else {
    cerr << "Object type not supported yet!\n";
    exit(1);
  }

  //Find degree of collision (i.e. maximum depth of penetration)
  if (objectC->type == OBJECT_HOLLOW_BOX) {
    HollowBoxObject *hb = (HollowBoxObject *)objectC;

    vector<BoxObject> sides;

    BoxObject side1;
    side1.type = OBJECT_BOX;
    side1.pose.pos = vector3(hb->radius1-hb->thickness/2, 0, 0);
    side1.radius1 = hb->thickness/2;
    side1.radius2 = hb->radius2;
    side1.radius3 = hb->radius3;

    if (hb->sideOpen != 1) {
      sides.push_back(side1);
    }

    BoxObject side2;
    side2.type = OBJECT_BOX;
    side2.pose.pos = vector3(-hb->radius1+hb->thickness/2, 0, 0);
    side2.radius1 = hb->thickness/2;
    side2.radius2 = hb->radius2;
    side2.radius3 = hb->radius3;

    if (hb->sideOpen != 2) {
      sides.push_back(side2);
    }

    BoxObject side3;
    side3.type = OBJECT_BOX;
    side3.pose.pos = vector3(0, hb->radius2-hb->thickness/2, 0);
    side3.radius1 = hb->radius1;
    side3.radius2 = hb->thickness/2;
    side3.radius3 = hb->radius3;

    if (hb->sideOpen != 3) {
      sides.push_back(side3);
    }


    BoxObject side4;
    side4.type = OBJECT_BOX;
    side4.pose.pos = vector3(0, -hb->radius2+hb->thickness/2, 0);
    side4.radius1 = hb->radius1;
    side4.radius2 = hb->thickness/2;
    side4.radius3 = hb->radius3;

    if (hb->sideOpen != 4) {
      sides.push_back(side4);
    }

    BoxObject side5;
    side5.type = OBJECT_BOX;
    side5.pose.pos = vector3(0, 0, hb->radius3+hb->thickness/2);
    side5.radius1 = hb->radius1;
    side5.radius2 = hb->radius2;
    side5.radius3 = hb->thickness/2;

    if (hb->sideOpen != 5) {
      sides.push_back(side5);
    }

    BoxObject side6;
    side6.type = OBJECT_BOX;
    side6.pose.pos = vector3(0, 0, -hb->radius3+hb->thickness/2);
    side6.radius1 = hb->radius1;
    side6.radius2 = hb->radius2;
    side6.radius3 = hb->thickness/2;

    if (hb->sideOpen != 6) {
      sides.push_back(side6);
    }

    if (objectO->type == OBJECT_BOX || objectO->type == OBJECT_HOLLOW_BOX) {
      double penetrationPenalty = 0.0;
      vector<Vector3> patch;
      for (unsigned int i = 0; i < sides.size(); i++) {
	setIdentity(sides[i].pose.rot);
	transform(hb->pose, sides[i].pose, sides[i].pose);
	Witness witness = findContactPatch(sides[i], *((BoxObject*)objectO),
	    &patch, 0);
	if (witness.distance < 0) {
	  penetrationPenalty -= witness.distance;
	}
      }
      return containedVolume / entireVolume *
	exp(-penetrationPenalty/distanceFalloffInside * 0.3010129996);
    }
    else {
      cerr << "Trajector object type not yet supported!\n";
      exit(1);
    }
  }
    
  return containedVolume / entireVolume;
}

};
