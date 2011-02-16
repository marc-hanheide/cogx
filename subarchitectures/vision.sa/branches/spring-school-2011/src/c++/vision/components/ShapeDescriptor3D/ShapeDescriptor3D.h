/**
 * @author Michael Zillich
 * @date September 2010
 */

#ifndef SHAPE_DESCRIPTOR_3D_H
#define SHAPE_DESCRIPTOR_3D_H

#include <cast/architecture/ManagedComponent.hpp>
#include <StereoClient.h>
#include <VisionData.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif
#include "DShapeCore.hh"
#include "PlanarPatch.h"

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;

class ShapeDescriptor3D : public StereoClient,
                          public ManagedComponent
{
private:
  /**
   * image width at which the stereo server should operate for us.
   * smaller image are of course faster
   */
  int stereoWidth;
  /**
   * size of the angle histogram of the RAS shape descriptor
   */
  int histogramSize;
  /**
   * actual shape description class
   */
  P::DShapeCore dshape;
  /**
   * current proto object
   */
  ProtoObjectPtr pobjPtr;
  /**
   * if true, use stereo matching of keypoints and plane detection based on homographies
   * if false, use 3D points from stereo point cloud and detect planes in 3D
   * The latter tends to get better results when there is no strong "keypoint-ish" texture,
   * where no keypoints are found but dense stereo still has enough texture to reconstruct
   * a proper surface.
   */
  bool useKeyPoints;
  /**
   * current set of planes detected in 3D points (only used if useKeypoints == false)
   */
  P::Array<P::Plane*> rasPlanes;
  double ransacThr;
  int planeMinPoints;
  bool findConnectedRegions;
  PlaneRANSAC *planeDetector;
  /**
   * whether to log images to files
   */
  bool logImages;

#ifdef FEAT_VISUALIZATION
	class MyDisplayClient: public display::CDisplayClient
	{
		ShapeDescriptor3D *owner;
	public:
		MyDisplayClient() { owner = 0; }
		void setClientData(ShapeDescriptor3D *_owner) { owner = _owner; }
		void handleEvent(const Visualization::TEvent &event);
		std::string getControlState(const std::string &ctrlId);
	};
	MyDisplayClient m_display;
  map<unsigned, P::RGBColor> displayColors;

  void redrawHistogram(const ProtoObject &pobj);
  void redraw3D();
#endif

  /**
   * callback function called whenever a new ProtoObject appears
   */
  void newProtoObject(const cdl::WorkingMemoryChange & _wmc);
  /**
   * callback function called whenever a ProtoObject changes
   */
  void updatedProtoObject(const cdl::WorkingMemoryChange & _wmc);

  void calculateDescriptor(ProtoObject &pobj);
  
  void calculateDescriptor2(ProtoObject &pobj);

  void findPlanes(vector<PlanarPatch> &planes, SurfacePointSeq &points, int &n);

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
	ShapeDescriptor3D();
  virtual ~ShapeDescriptor3D();
};

}

#endif



