#include "PTZServer.hpp"

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
  

  PTZServer::PTZServer()
#ifdef FEAT_VISUALIZATION
    : mDisplay(this)
#endif
  {
  }

  void PTZServer::configure(const std::map<std::string,std::string> & _config)  {
    //setup ice server
    PTZInterfacePtr servant = new PTZServerI(this);
    registerIceServer<PTZInterface, PTZInterface>(servant);

#ifdef FEAT_VISUALIZATION
    display().configureDisplayClient(_config);
#endif
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

  void PTZServer::runComponent()
  {
#ifdef FEAT_VISUALIZATION
    const double epsmove = 1e-3; // [radians] the min angle distance that is treated as a movement
    const int intervalMs = 100;  // time between checks
    const int waitStartMove = 2; // number of time intervals to wait before start-move is accepted
    const int waitEndMove = 3;   // number of time intervals to wait before end-move is accepted

    sendPtuStateToDialog();
    PTZPose pose = getPose().pose;
    PTZPose oldpose = getPose().pose;
    bool bMoving = false;
    int changeWait = 0;
    mbPoseWasSet = false;
    while (isRunning()) {
      sleepComponent(intervalMs);
      if (mbPoseWasSet) {
        mbPoseWasSet = false;
        bMoving = true;
        changeWait = waitEndMove;
      }
      PTZPose pose = getPose().pose;
      double delta = fabs(pose.pan - oldpose.pan) + fabs(pose.tilt - oldpose.tilt);
      if (bMoving) {
        // detect end-of-move and update ptu-ctrl
        if (delta >= epsmove) {
          // still moving
          oldpose = pose;
          changeWait = waitEndMove;
        }
        else {
          --changeWait;
          if (changeWait <= 0) {
            sendPtuStateToDialog();
            bMoving = false;
            changeWait = waitStartMove;
          }
          //log("changeWait %d while stopping", changeWait);
        }
      }
      else {
        // detect start-of-move
        if (delta >= epsmove) {
          // started moving
          --changeWait;
          if (changeWait <= 0) {
             oldpose = pose;
             bMoving = true;
             changeWait = waitEndMove;
          }
        }
        else {
           // still near the same position
           changeWait = waitStartMove;
        }
      }
    }
#endif
  }

#ifdef FEAT_VISUALIZATION

  void PTZServer::sendPtuStateToDialog()
  {
    log("PtuCtrl: sendStateToDialog");
    PTZReading ptup = getPose();
    std::ostringstream ss;
    ss << "ptuctrl.ui.wctrls.spinPan.value=" << ptup.pose.pan * 180 / M_PI << ";";
    ss << "ptuctrl.ui.wctrls.spinTilt.value=" << ptup.pose.tilt * 180 / M_PI << ";";
    ss << "ptuctrl.ui.wctrls.spinZoom.value=" << ptup.pose.zoom << ";";
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
        mpPtzServer->sendPtuStateToDialog();
    }
  }

#endif
}
