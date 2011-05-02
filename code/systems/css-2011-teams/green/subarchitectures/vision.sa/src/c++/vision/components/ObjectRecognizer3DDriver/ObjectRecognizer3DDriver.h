/**
 * @author Thomas Mörwald
 * @date Februar 2010
 *
 * Just a dummy component to drive some vision functionality.
 */

#ifndef OBJECT_RECOGNIZER3D_DRIVER_H
#define OBJECT_RECOGNIZER3D_DRIVER_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <manipulation.hpp>
#include <PTZ.hpp>

#include "ObjectTrackerUtils.hpp"
#include "ModelLoader.h"
#include "Timer.h"


namespace cast
{

class ObjectRecognizer3DDriver : public ManagedComponent
{
private:
	std::string m_plyfile;
	std::vector<std::string> m_labels;
	std::vector<std::string> m_visualObjectIDs;
	std::map<std::string,int> m_sumDetections;
	std::map<std::string,float> m_sumConfidence;

	VisionData::Recognizer3DCommandPtr m_rec_cmd;
	manipulation::slice::FarArmMovementCommandPtr m_arm_cmd;

	std::string m_manipulation_sa;
	int m_mode;
	int m_loops;
	bool m_halt_rec;
	bool m_halt_arm;
	Timer m_timer;
	
	bool m_ptz;

	/** @brief receiving visual objects */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

  /** @brief read result of a recognition command */
  void overwriteRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc);
  
  void overwriteSetPTZPoseCommand(const cdl::WorkingMemoryChange & _wmc);

  /** @brief read result of a arm movement command */
  void overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc);

  /** @brief read result of a arm movement command */
  void overwriteCloseGripperCommand(const cdl::WorkingMemoryChange & _wmc);

  /** @brief loads ply from file and adds it into working memory */
  void loadVisualModelToWM(std::string filename, std::string& modelID, cogx::Math::Pose3 pose);

	/** @brief constructs a Recognizer3DCommand and adds it into working memory */
	void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID);

	/** @brief constructs a TrackingCommand and adds it into working memory */
	void addTrackingCommand(VisionData::TrackingCommandType cmd);
	
	void addPTZCommand(double pan, double tilt);

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
  virtual ~ObjectRecognizer3DDriver() {}
};

}

#endif

