/**
 * @author Alen Vrecko
 * @date September 2011
 *
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */

#ifndef ARM_MANAGER_H
#define ARM_MANAGER_H

#define POINTING_OFFSET 0.3

#include <cast/architecture/ManagedComponent.hpp>

#ifdef FEAT_VISUALIZATION
//#include <CDisplayClient.hpp>
#endif

#include <manipulation.hpp>
#include <manipulation_exe.hpp>

#include <VisionData.hpp>
#include <Pose3.h>

#include <queue>
#include <IceUtil/IceUtil.h>

namespace cogx
{

using namespace std;
using namespace cast;
using namespace manipulation::slice;
using namespace manipulation::execution::slice;

/**
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */
class ArmManager : public ManagedComponent
{
private:
	bool m_halt_arm;
	bool m_repeat_arm_movement;
	bool m_pointing_now;
	
	cdl::WorkingMemoryAddress m_pointedObjAddr;
	
/*	
	struct armAction {
		ManipulationTaskType type;
		cdl::WorkingMemoryAddress objAddr;
	};
*/		
	std::queue<cdl::WorkingMemoryAddress> m_actionQueue;
	IceUtil::Monitor<IceUtil::Mutex> m_queueMonitor;

#ifdef FEAT_VISUALIZATION
/*  class PABDisplayClient : public cogx::display::CDisplayClient
  {
  private:
    ArmManager *m_comp;
  public:
    cogx::display::CFormValues m_frmSettings;

  public:
    PABDisplayClient() { m_comp = 0; }
    void setClientData(ArmManager *comp) { m_comp = comp; }
    /*void handleEvent(const Visualization::TEvent &event);
    std::string getControlState(const std::string& ctrlId);
    void handleForm(const std::string& id, const std::string& partId,
          const std::map<std::string, std::string>& fields);
    bool getFormData(const std::string& id, const std::string& partId,
          std::map<std::string, std::string>& fields);//
  };
  PABDisplayClient display; */
#endif
  bool pointAtObject(cdl::WorkingMemoryAddress addr);
  cogx::Math::Pose3 pointingPose(const cogx::Math::Pose3 objPose);
  
  bool addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma); //, cogx::Math::Vector3 offset);
  bool addMoveToHomeCommand();
  bool addMoveArmToPose(cogx::Math::Pose3 pose);
  
  void receiveNewCommand(const cdl::WorkingMemoryChange &_wmc);
  void receiveDeletedObject(const cdl::WorkingMemoryChange &_wmc);
  void overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc);
  void overwriteMoveToHomeCommand(const cdl::WorkingMemoryChange & _wmc);
  void overwriteMoveToPose(const cdl::WorkingMemoryChange & _wmc);
  
protected:
  virtual void configure(const map<string, string> &_config);
  virtual void start();
  virtual void destroy();
  virtual void runComponent();

public:
  ArmManager();
};

}

#endif
