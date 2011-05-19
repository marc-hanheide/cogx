/**
 * @file StereoDetectorReasoner.h
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Reasoning about detected planes and detected visual objects.
 */

#ifndef SDR_H
#define SDR_H

#include <cast/architecture/ManagedComponent.hpp>

#include <vector>
#include <VisionData.hpp>
#include "Vector.hh"
#include "cogxmath.h"
#include "Plane.h"
#include "Object.h"

namespace cast
{

using namespace VisionData;

/**
 * @class StereoDetectorReasoner
 * @brief Try to explain the scene, based on detected stereo data.
 */
class StereoDetectorReasoner : public ManagedComponent
{
private:
// 	struct SDRObjects
// 	{
// 		std::vector<Z::Object*> obj;
// 	};
//	std::vector<SDRObjects> vObjects;			///< stored objects

	Z::Plane *plane;															///< The ground plane													/// TODO es sollten mehrere planes gespeichert bleiben, wie bei objects
	bool havePlane;																///< true, if a plane is stored
	std::string planeID;													///< Plane-address on working memory
	std::vector<std::string> objectIDs;						///< Object-addresses on working memory
	
	void receiveConvexHull(const cdl::WorkingMemoryChange & _wmc);
	void deleteConvexHull(const cdl::WorkingMemoryChange & _wmc);
	void receiveReasonerObject(const cdl::WorkingMemoryChange & _wmc);
	void receiveSDReasonerCommand(const cdl::WorkingMemoryChange & _wmc);
	
protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  StereoDetectorReasoner() {}
  virtual ~StereoDetectorReasoner();
};

}

#endif



