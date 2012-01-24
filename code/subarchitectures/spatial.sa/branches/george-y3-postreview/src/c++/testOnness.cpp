#include "RelationEvaluation.hpp"
#include <Navigation/LocalGridMap.hh>
#include "DensitySampling.hpp"

// gcc -pg -o test -I$CURE_ROOT/src -Iautogen -I../../../../tools/math/src/c++/autogen/ -I../../../../tools/math/src/c++/math -L$CURE_ROOT/lib/cure -L/usr/local/lib/cast -L../../../../output/lib -lCureTransformation -lCureMath -lCureUtils -lCureGeometry -lCureAddressBank -lCureFilters -lCureSensory -lCureSensorData -lpthread -lIce -lCDL -lCASTCore -lMath testOnness.cpp DensitySampling.cpp RelationEvaluation.cpp OnnessEvaluation.cpp InnessEvaluation.cpp

using namespace spatial;
using namespace std;

int main(void) 
{
  PlaneObject table1;
  table1.type = OBJECT_PLANE;
  //table1.pose = pose3(vector3(0.0, 0.0, 1.0), rotation);

  table1.shape = PLANE_OBJECT_RECTANGLE;
  table1.radius1 = 0.5;
  table1.radius2 = 0.5;

  BoxObject box1;

  box1.type = OBJECT_BOX;
//  box1.pose = boxPose;
  box1.radius1 = 0.095;
  box1.radius2 = 0.045;
  box1.radius3 = 0.145;

  HollowBoxObject box2;

  box2.type = OBJECT_HOLLOW_BOX;
//  box2.pose = boxPose;
  box2.radius1 = 0.115;
  box2.radius2 = 0.105;
  box2.radius3 = 0.13;
  box2.thickness = 0.02;

  DensitySampler sampler;


  vector<Matrix33> oris1;
  oris1.push_back(Matrix33());
  oris1.back().m00 = 1;
  oris1.back().m11 = 1;
  oris1.back().m22 = 1;
//  vector<Matrix33> oris2;
//  getRandomSampleSphere(oris2, 3);
//  vector<Matrix33> oris3;
//  getRandomSampleSphere(oris3, 3);

  Cure::LocalGridMap<double> pdf(25, 0.05, 0.0, 
      Cure::LocalGridMap<double>::MAP1, 0, 0);

  vector<spatial::Object *>objects;
  vector<spatial::SpatialRelationType> relations;
  vector<string> labels;
  objects.push_back(&box1);
  labels.push_back("box1");

  bool sampleTable = false;

  if (sampleTable) {
    objects.push_back(&table1);
    relations.push_back(RELATION_ON);
  }
  else {
    objects.push_back(&box2);
    labels.push_back("box2");
    relations.push_back(RELATION_ON);
    objects.push_back(&table1);
    labels.push_back("table1");
    relations.push_back(RELATION_ON);
  }

  //	    objects.push_back(&box2);
  //	    relations.push_back(RELATION_ON);

  double total = 0.0;
  SampleCloud testCloud;
  sampler.sampleBinaryRelationSystematically(relations, objects, 
      oris1, labels,
      pdf.getCellSize(), testCloud);

  testCloud.KernelDensityEstimation2D(pdf, 
      objects.back()->pose.pos, sampler.m_kernelWidthFactor, 
      total, 1.0);
}
