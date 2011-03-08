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

#include <Navigation/LocalGridMap.hh>
#include "XVector3D.h"
#include "Math/BinaryMatrix.hh"
#include <SensorData/SensorPose.hh>
#include "BloxelFunctors.hh"
#include "GridDataFunctors.hh"

namespace spatial {

class AVS_ContinualPlanner;

class ViewPointGenerator {

public:
	typedef Cure::LocalGridMap<unsigned char> CureObstMap;
	typedef SpatialGridMap::GridMap<SpatialGridMap::GridMapData> BloxelMap;
	typedef Cure::LocalGridMap<double> CurePDFMap;

	struct SensingAction {
			std::vector<double> pos;
			double pan;
			double tilt;
			double totalprob;
			double conedepth;
			double horizangle, vertangle, minDistance;
	};

	ViewPointGenerator(AVS_ContinualPlanner* component, CureObstMap* plgm, BloxelMap* pbloxelmap, int samplesize,
			double sampleawayfromobs, double conedepth, double horizangle, double vertangle,
			double minDistance, double pdfsum, double pdfthreshold, double robotx, double roboty);
	virtual ~ViewPointGenerator();

	vector<pair<unsigned int, double> > get2DCandidateViewCones();

	bool isPointSameSide(XVector3D p1, XVector3D p2, XVector3D a, XVector3D b);
	void findBoundingRectangle(XVector3D a, XVector3D b, XVector3D c,
			int* rectangle);
	void get2DViewConeCorners(XVector3D a, double direction, double range,
			double fov, XVector3D &b, XVector3D &c);
	std::vector<pair<int, int> > getInside2DViewCone(CureObstMap* lgm,
			XVector3D &a, bool addall);
	std::vector<std::vector<pair<int, int> > > calculate3DViewConesFrom2D();
	std::vector<Cure::Pose3D> sample2DGrid();
	double getPathLength(Cure::Pose3D start, Cure::Pose3D destination,
			CureObstMap* lgm);
	bool isCircleFree2D(const CureObstMap &map, double xW, double yW,
			double rad);
	bool isPointInsideTriangle(XVector3D p, XVector3D a, XVector3D b,
			XVector3D c);

	std::vector<SensingAction> getViewConeSums(std::vector <SensingAction > &samplepoints);

	vector<ViewPointGenerator::SensingAction> getBest3DViewCones();

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
    double m_tiltinterval;
    double m_bloxelmapPDFsum;
    double m_pdfthreshold;

};
}
;
#endif /* VIEWPOINTGENERATOR_H_ */
