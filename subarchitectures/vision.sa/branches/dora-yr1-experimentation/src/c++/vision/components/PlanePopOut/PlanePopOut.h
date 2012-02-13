/**
 * @author Kai ZHOU
 * @date June 2009
 *
 * Just receives stereo point clouds and displays them.
 */

#ifndef PLANE_POPOUT_H
#define PLANE_POPOUT_H

#include <cast/architecture/ManagedComponent.hpp>
#include <StereoClient.h>
#include <VisionData.hpp>


namespace cast
{
using namespace cogx;
using namespace cogx::Math;

class PlanePopOut : public StereoClient,
                     public ManagedComponent
{
typedef struct ObjP
{
	Vector3 c;
	Vector3 s;
	double r;
	std::string id;
	VisionData::SurfacePointSeq pointsInOneSOI;
	VisionData::SurfacePointSeq BGInOneSOI;
	VisionData::SurfacePointSeq EQInOneSOI;
}ObjPara;

private:
  /**
   * Which camera to get images from
   */
  int camId;
  /**
   * whether to use stereo points in global or left camear coord system.
   */
  bool useGlobalPoints;
	double Calc_SplitThreshold(VisionData::SurfacePointSeq &points, std::vector <int> &label);
	std::vector<ObjPara> PreviousObjList;
	std::vector<ObjPara> CurrentObjList;
	VisionData::SOIPtr createObj(Vector3 center, Vector3 size, double radius, VisionData::SurfacePointSeq psIn1SOI, VisionData::SurfacePointSeq BGpIn1SOI, VisionData::SurfacePointSeq EQpIn1SOI);
	bool Compare2SOI(ObjPara obj1, ObjPara obj2);
	void AddConvexHullinWM();


protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

public:


	bool RANSAC(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	void SplitPoints(VisionData::SurfacePointSeq &points, std::vector <int> &labels);

	double para_a;
	double para_b;
	double para_c;
	double para_d;


	PlanePopOut() : camId(0) 
	{
		para_a = 0.0;
		para_b = 0.0;
		para_c = 0.0;
		para_d = 0.0;
	}
  virtual ~PlanePopOut() {}
};

}

#endif


