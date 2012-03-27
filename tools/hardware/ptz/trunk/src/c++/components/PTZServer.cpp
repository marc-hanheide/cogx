#include "PTZServer.hpp"
#include "Timers.hpp"

#include <sstream>
#include <cmath>

using namespace cast;

namespace ptz {

  ptz::PTZReading PTZServerI::getPose(const Ice::Current & _crt) const {
    return m_ptzServer->getPose();
  }
  
  void PTZServerI::setPose(const ptz::PTZPose & _pose, 
			   const Ice::Current & _crt) {
    m_ptzServer->mbPoseWasSet = true;
    return m_ptzServer->setPose(_pose);
  }
  
  bool PTZServerI::isMoving(const Ice::Current & _crt) const {
    return m_ptzServer->mbMoving;
  }

  PTZServer::PTZServer()
#ifdef FEAT_VISUALIZATION
    : mDisplay(this)
#endif
  {
     mbMoving = false;
     mbPoseWasSet = false;
     mMotionTollerance = 1e-4;
  }

  void PTZServer::configure(const std::map<std::string,std::string> & _config)  {
     std::map<std::string,std::string>::const_iterator it;

    // default: 1e-4
    // for gazebo simulation: set to at least 2e-3
    if((it = _config.find("--motion_tollerance")) != _config.end())
    {
      std::istringstream str(it->second);
      str >> mMotionTollerance;
      if (mMotionTollerance < 1e-9) mMotionTollerance = 1e-9;
      if (mMotionTollerance > 0.1) mMotionTollerance = 0.1;
    }
    log("Motion Tollerance: %.9f", mMotionTollerance);

#ifdef FEAT_VISUALIZATION
    display().configureDisplayClient(_config);
#endif

    //setup ice server
    PTZInterfacePtr servant = new PTZServerI(this);
    registerIceServer<PTZInterface, PTZInterface>(servant);
  }

#ifdef FEAT_VISUALIZATION
#include "res/ptucontrol.inc"
#endif

  void PTZServer::start()
  {
#ifdef FEAT_VISUALIZATION
    display().connectIceClient(*this);
    display().installEventReceiver();
    display().mDialogId = getComponentID() + "#PtuControl";
    display().addDialog(display().mDialogId, res_ptucontroller_ui, res_ptucontroller_js, "PtuController ptuctrl");
#endif
  }

  // TODO: check the actual frequency of the loop; use rad/sec for motion tollerance (instead of rad)
  void PTZServer::runComponent()
  {
    const int intervalMs = 100;  // time between checks
    const int waitStartMove = 2; // number of time intervals to wait before start-move is accepted
    const int waitEndMove = 3;   // number of time intervals to wait before end-move is accepted
    double epsmove = mMotionTollerance * intervalMs / 1000;
    CCastPaceMaker<PTZServer> pace(*this, intervalMs, 1);
    CMilliTimer tmInfo;

    log("eps: %.8f", epsmove);
#ifdef FEAT_VISUALIZATION
    sendPtuStateToDialog();
#endif
    PTZPose pose = getPose().pose;
    PTZPose oldpose = getPose().pose;
    int changeWait = waitStartMove;
    mbPoseWasSet = false;
    mbMoving = false;
    while (isRunning()) {
      pace.sync();
      epsmove = mMotionTollerance / pace.getTotalRate();

      if (mbPoseWasSet) {
        mbPoseWasSet = false;
        mbMoving = true;
        changeWait = waitEndMove;
        log("Start of move (setPose).");
      }

      PTZPose pose = getPose().pose;
      double delta = fabs(pose.pan - oldpose.pan) + fabs(pose.tilt - oldpose.tilt);

      if (tmInfo.elapsed() > 5000) {
#ifdef FEAT_VISUALIZATION
        std::ostringstream ss;
        ss.precision(6);
        ss << "<h3>PTZ Server</h3>"
          << "Running rate: " << pace.getTotalRate() << "hz<br>"
          << "Motion tollerance: " << mMotionTollerance << "rad/s<br>"
          << "Actual tollerance: " << epsmove << "rad";
        display().setHtml("INFO", "PTZ.Server" + getComponentID(), ss.str());
#else
        log("rate: %.3f, delta: %.8f, eps: %.8f", pace.getTotalRate(), delta, epsmove);
#endif
        tmInfo.restart();
      }

      if (mbMoving) {
        // detect end-of-move and update ptu-ctrl
        if (delta >= epsmove) {
          // still moving
          oldpose = pose;
          changeWait = waitEndMove;
        }
        else {
          --changeWait;
          if (changeWait <= 0) {
            mbMoving = false;
            changeWait = waitStartMove;
            log("End of move.");
#ifdef FEAT_VISUALIZATION
            sendPtuStateToDialog();
#endif
          }
        }
      }
      else {
        // detect start-of-move
        if (delta >= epsmove) {
          // started moving
          --changeWait;
          if (changeWait <= 0) {
            oldpose = pose;
            mbMoving = true;
            changeWait = waitEndMove;
            log("Start of move.");
          }
        }
        else {
          // still near the same position
          changeWait = waitStartMove;
        }
      }
    }
  }

#ifdef FEAT_VISUALIZATION

  void PTZServer::sendPtuStateToDialog(bool bForce)
  {
    //log("PtuCtrl: sendStateToDialog");
    PTZReading ptup = getPose();
    std::ostringstream ss;
    ss << "ptuctrl.setPtzPosition(" 
       << ptup.pose.pan * 180 / M_PI << ", "
       << ptup.pose.tilt * 180 / M_PI << ", "
       << ptup.pose.zoom << ", "
       << (bForce ? "true" : "false")
       << ")";
    //log(ss.str());
    display().execInDialog(display().mDialogId, ss.str());
  }

  PTZServer::CDisplayClient::CDisplayClient(PTZServer* pPtzServer)
    :mpPtzServer(pPtzServer)
  {
  }

  void PTZServer::CDisplayClient::onDialogValueChanged(const std::string& dialogId,
      const std::string& name, const std::string& value)
  {
    if (dialogId == mDialogId) {
      if (name == "PTZ") {
        println(" *** PTZ command from dialog *** ");
        double pan, tilt, zoom;
        int nf = sscanf(value.c_str(), "%lf, %lf, %lf", &pan, &tilt, &zoom);
        if (nf == 3) {
          PTZPose pose;
          pose.pan = pan * M_PI / 180;
          pose.tilt = tilt * M_PI / 180;
          pose.zoom = zoom;
          mpPtzServer->setPose(pose);
        }
      }
    }
  }

  void PTZServer::CDisplayClient::handleDialogCommand(const std::string& dialogId,
        const std::string& command, const std::string& params)
  {
    if (dialogId == mDialogId) {
      //println(" *** handleDialogCommand *** " + command);
      if (command == "sendStateToDialog")
        mpPtzServer->sendPtuStateToDialog(/*force=*/ true);
    }
  }

#endif
}
