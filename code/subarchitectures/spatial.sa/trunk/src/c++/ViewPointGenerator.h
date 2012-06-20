/*
 * ViewPointGenerator.h
 *
 *  Created on: Mar 4, 2011
 *      Author: alper
 */

#ifndef VIEWPOINTGENERATOR_H_
#define VIEWPOINTGENERATOR_H_

#include "SpatialGridMap.hh"
#include "GridMapData.hh"
#include <NavData.hpp>

#include "NewNavController.hpp"
#include <Navigation/LocalGridMap.hh>
#include "XVector3D.h"
#include "Math/BinaryMatrix.hh"
#include <SensorData/SensorPose.hh>
#include "BloxelFunctors.hh"
#include "GridDataFunctors.hh"
//#include "AVS_ContinualPlanner.h"

namespace spatial {

class AVS_ContinualPlanner;

class ViewPointGenerator {

public:
  typedef Cure::LocalGridMap<unsigned char> CureObstMap;
  typedef SpatialGridMap::GridMap<SpatialGridMap::GridMapData> BloxelMap;
  typedef Cure::LocalGridMap<double> CurePDFMap;
  typedef std::vector<std::pair<std::vector<double>, double> > posPDF; // <position,pdf>

  struct SensingAction {
    std::vector<double> pos;
    double pan;
    double tilt;
    double totalprob;
    double conedepth;
    double horizangle, vertangle, minDistance;
    posPDF pdfcache;
  };

  ViewPointGenerator(AVS_ContinualPlanner* component, CureObstMap* plgm,
      BloxelMap* pbloxelmap, int samplesize, double sampleawayfromobs,
      double conedepth, double tiltstep, double panstep, double horizangle,
      double vertangle, double minDistance, double pdfsum, double pdfthreshold,
      double robotx, double roboty);
  virtual ~ViewPointGenerator();
  double m_lastMapPDFSum;
  //NEW
  void findIntersectingCones2D();
  int TrianglesIntersecting(XVector3D p0, XVector3D p1, XVector3D p2,
      XVector3D t0, XVector3D t1, XVector3D t2);
  int Intersecting(XVector3D p0, XVector3D p1, XVector3D t0, XVector3D t1,
      XVector3D t2);
  float Side(XVector3D p, XVector3D q, XVector3D a, XVector3D b);
  map<int, vector<int> > m_viewconesIntersections;

  vector<pair<unsigned int, double> > getOrdered2DCandidateViewCones(vector<
      NavData::FNodePtr> &nodes);

  bool isPointSameSide(XVector3D p1, XVector3D p2, XVector3D a, XVector3D b);
  void findBoundingRectangle(XVector3D a, XVector3D b, XVector3D c,
      int* rectangle);
  void get2DViewConeCorners(XVector3D a, double direction, double range,
      double fov, XVector3D &b, XVector3D &c);
  std::vector<pair<int, int> > getInside2DViewCone(CureObstMap* lgm,
      XVector3D &a, bool addall);
  std::vector<std::vector<pair<int, int> > > calculate2DConesRegion();
  std::vector<Cure::Pose3D> sample2DGrid();
  double getPathLength(Cure::Pose3D start, Cure::Pose3D destination,
      CureObstMap* lgm);
  bool isCircleFree2D(const CureObstMap &map, double xW, double yW, double rad);
  bool
  isPointInsideTriangle(XVector3D p, XVector3D a, XVector3D b, XVector3D c);

  std::vector<SensingAction> getViewConeSums(
      std::vector<SensingAction> &samplepoints);

  vector<ViewPointGenerator::SensingAction> getBest3DViewCones(vector<
      NavData::FNodePtr> &nodes);

  std::vector<Cure::Pose3D> sample2DGridFromNodes(
      vector<NavData::FNodePtr> &nodes);

  AVS_ContinualPlanner * m_component;
  std::vector<Cure::Pose3D> m_samples2D;
  CureObstMap* lgm;
  BloxelMap* bloxelmap;
  int m_samplesize;

  double m_sampleawayfromobs;
  double m_conedepth;
  double m_horizangle, m_vertangle, m_minDistance;
  double m_robotx, m_roboty, m_robottheta;
  double m_best3DConeRatio;
  double m_tiltstep;
  double m_bloxelmapPDFsum;
  double m_pdfthreshold;
  double m_sensingProb;
  double m_panstep;

};
}
;
#endif /* VIEWPOINTGENERATOR_H_ */
