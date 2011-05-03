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

	manipulation::slice::FarArmMovementCommandPtr m_arm_cmd;
	manipulation::slice::MoveArmToPosePtr m_moveto_cmd;

	cast::cdl::WorkingMemoryAddress m_grasp_wma;
	VisionData::GraspForObjectCommandPtr m_grasp_cmd;
	VisionData::LookForObjectCommandPtr m_look_cmd;

	std::string m_latest_visualObjectID;

	std::string m_manipulation_sa;
	int m_mode;
	int m_loops;
	bool m_grasp;
	bool m_repeat_arm_movement;
	bool m_look;
	bool m_halt_rec;
	bool m_halt_arm;
	Timer m_timer;
	
	double m_obj_distance;
	int m_rec_objects;

	bool m_halt_ptz;
	double m_pan;
	double m_tilt;

	/** @brief looks for objects in a certain area */
	void receiveLookForObjectCommand(const cdl::WorkingMemoryChange & _wmc);

	void receiveGraspForObjectCommand(const cdl::WorkingMemoryChange & _wmc);

	/** @brief read result of a recognition command */
	void overwriteRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc);

	void overwriteSetPTZPoseCommand(const cdl::WorkingMemoryChange & _wmc);

	void overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc);
	void overwriteMoveArmToPose(const cdl::WorkingMemoryChange & _wmc);
	void overwriteCloseGripperCommand(const cdl::WorkingMemoryChange & _wmc);
	void overwriteOpenGripperCommand(const cdl::WorkingMemoryChange & _wmc);

	/** @brief loads ply from file and adds it into working memory */
	void loadVisualModelToWM(std::string filename, std::string& modelID, cogx::Math::Pose3 pose);

	void addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma, cogx::Math::Vector3 vOffset);
	void addMoveArmToPoseCommand(cogx::Math::Pose3 pose);
	void addOpenGripperCommand();
	void addCloseGripperCommand();

	void doLooking();
	void doGrasping();

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

