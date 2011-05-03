#ifndef BLUE_FSM_H
#define BLUE_FSM_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <manipulation.hpp>
#include <boost/thread/mutex.hpp>
#include <Navigation/LocalGridMap.hh>
#include <FrontierInterface.hpp>
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <Navigation/LocalMap.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <SensorData/LaserScan2d.hh>
#include <CureHWUtils.hpp>

typedef Cure::LocalGridMap<unsigned char> CharMap ;
typedef Cure::GridLineRayTracer<unsigned char> CharGridLineRayTracer;

namespace cogx
{
  typedef Cure::LocalGridMap<unsigned char> CureObstMap;
  
  using namespace std;
  using namespace cast;
  
  class BlueFSM : public ManagedComponent,
  		  public OdometryReceiver,
		  public Scan2dReceiver
  {
    enum State {INIT,
      SPINNING,
      MOVE_TO_NEW_POS,
      DECIDE_POSITION,
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
      DELIVER_TO_HOME_POSITION,
      DELIVER_TO_DROPOFF_TABLE,
      TERMINATED};
  private:
    
  protected:
    virtual void configure(const map<string, string> &_config);
    virtual void start();
    virtual void destroy();
    virtual void runComponent();

    void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID);

    bool moveHome();
    bool moveToSafePose();
    bool movePregrasp(cogx::Math::Pose3 pregraspPose);
    bool envelop();
    bool lift();
    bool moveToHandover();
    bool retract();
    bool grasp();
    bool release();
    bool turn45Degrees();
    bool turnTo(double theta);
    bool moveTo(double x, double y, double theta = FLT_MAX);
    bool movePTZ(double pan, double tilt);
    
    void objectPoseCallback(const cdl::WorkingMemoryChange &_wmc);

    void simpleCallback(const cdl::WorkingMemoryChange &_wmc);
    void navCallback(const cdl::WorkingMemoryChange &_wmc);
    void ptzCallback(const cdl::WorkingMemoryChange &_wmc);

    void nameless(const Math::Pose3& inRobotPose,
                                            const Math::Pose3& inObjectPose,
                                            const std::string& inObjectLabel,
                                            Math::Pose3& outPregraspPose,
                                            Math::Pose3& outEnvelopingPose,
                                            double& outQuality) const;

    State m_state;
    bool m_waiting;

    bool m_dropTableDetected;

    int m_turnStep;

    boost::mutex mutex_;

    map<string, cogx::Math::Pose3> m_poses;
    map<string, cogx::Math::Pose3> m_globalPoses;
    map<string, double> m_pose_confs;

    bool m_useDropoff;
    vector<string> m_lookForObjects;

    cogx::Math::Pose3 m_handoverPose;
    cogx::Math::Pose3 m_pregraspPose;
    cogx::Math::Pose3 m_envelopingPose;
    cogx::Math::Pose3 m_currentArmPose;

    //SLAM related
    Cure::SensorPose m_LaserPoseR;
    Cure::TransformedOdomPoseProvider m_TOPP;
    CharMap* m_lgm;
    CharGridLineRayTracer* m_Glrt;
    Math::Pose3 m_CurrPose;

  private:
    void receiveScan2d(const Laser::Scan2d &castScan);
    void receiveOdometry(const Robotbase::Odometry &castOdom);
    bool findBestGraspPose(const string &obj, double &bestX, double &bestY, 
	double &bestTheta);
    void findRandomPosition(double &x, double &y);

    FrontierInterface::PlaceInterfacePrx m_placeInterface;
    FrontierInterface::LocalMapInterfacePrx m_mapInterface;

  public:
    BlueFSM();
    ~BlueFSM();
  };
  
}

#endif
