#ifndef BLUE_FSM_H
#define BLUE_FSM_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <manipulation.hpp>
#include <boost/thread/mutex.hpp>
#include <Navigation/LocalGridMap.hh>
#include <FrontierInterface.hpp>

namespace cogx
{
  typedef Cure::LocalGridMap<unsigned char> CureObstMap;
  
  using namespace std;
  using namespace cast;
  
  class BlueFSM : public ManagedComponent
  {
    enum State {INIT,
      SPINNING,
      MOVE_TO_NEW_POS,
      LOOK_CANONICAL, //Pan-tilt in calibrated position
      LOOK_AROUND,    //Pan-tilt ambulatory
      DETECTING,
      DECIDE_GRASP,   //Determine which grasp to use
      GO_TO_PREGRASP, 
      VERIFY_PREGRASP,//Check arm's position is correct
      ENVELOP,	      //Move arm forward
      VERIFY_ENVELOP,
      RETRACT,
      GRASP,	      //Close gripper
      VERIFY_GRASP,
      LIFT,
      HANDOVER,
      VERIFY_HANDOVER,
      RELEASE,
      GO_HOME,
      TERMINATED};
  private:
    
  protected:
    virtual void configure(const map<string, string> &_config);
    virtual void start();
    virtual void destroy();
    virtual void runComponent();

    void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID);

    bool moveHome();
    bool movePregrasp(cogx::Math::Pose3 pregraspPose);
    bool envelop();
    bool lift();
    bool moveToHandover();
    bool retract();
    bool grasp();
    bool release();
    
    void objectPoseCallback(const cdl::WorkingMemoryChange &_wmc);
    void simpleCallback(const cdl::WorkingMemoryChange &_wmc);

    std::pair<Math::Pose3, double> nameless(const Math::Pose3& inRobotPose,
                                            const Math::Pose3& inObjectPose,
                                            const std::string& inObjectLabel,
                                            Math::Pose3& outPregraspPose,
                                            Math::Pose3& outEnvelopingPose,
                                            double& outQuality) const;

    State m_state;
    bool m_waiting;

    int m_turnStep;

    boost::mutex mutex_;

    map<string, cogx::Math::Pose3> m_poses;
    map<string, double> m_pose_confs;

    vector<string> m_lookForObjects;

    cogx::Math::Pose3 m_handoverPose;
    cogx::Math::Pose3 m_pregraspPose;
    cogx::Math::Pose3 m_envelopingPose;
    cogx::Math::Pose3 m_currentArmPose;
    
//    PTZInterfacePrx m_ptzInterface;
  private:
    bool findGraspPoses(double objX, double objY, double theta, double halfLength,
	double bestX, double bestY);

    FrontierInterface::PlaceInterfacePrx m_placeInterface;
    FrontierInterface::LocalMapInterfacePrx m_mapInterface;

  public:
    BlueFSM();
  };
  
}

#endif
