/**
 * @author Michael Zillich
 * @date April 2011
 *
 * Uses the player/gazebo simulation interface to continuosly rotate a given
 * object. Useful for learning virtual objects.
 */

#ifndef GAZEBO_TURNTABLE_H
#define GAZEBO_TURNTABLE_H

#include <cast/architecture/ManagedComponent.hpp>
#include <libplayerc++/playerc++.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cogx
{

using namespace std;
using namespace cast;

/**
 * Uses the player/gazebo simulation interface to continuosly rotate a given
 * object. Useful for learning virtual objects.
 */
class GazeboTurntable : public ManagedComponent
{
private:
#ifdef FEAT_VISUALIZATION
  class GTDisplayClient : public cogx::display::CDisplayClient
  {
  private:
    GazeboTurntable *m_comp;
  public:
    cogx::display::CFormValues m_frmSettings;

  public:
    GTDisplayClient() { m_comp = 0; }
    void setClientData(GazeboTurntable *comp) { m_comp = comp; }
    void handleEvent(const Visualization::TEvent &event);
    std::string getControlState(const std::string& ctrlId);
    /*void handleForm(const std::string& id, const std::string& partId,
          const std::map<std::string, std::string>& fields);
    bool getFormData(const std::string& id, const std::string& partId,
          std::map<std::string, std::string>& fields);*/
  };
  GTDisplayClient display;
#endif

  /**
   * host on which player server runs (typically localhost)
   */
  string playerHost;
  /**
   * player port number (typically 6665)
   */
  uint32_t playerPort;
  /** player client object */
  PlayerCc::PlayerClient *robot;
  /** interface to the simulation */
  PlayerCc::SimulationProxy *sim;
  /**
   * name of object in gazebo world, which we want to rotate
   */
  string objLabel;
  /** rotation speed in rad/s */
  double rotSpeed;
  /** time between steps in ms */
  int timeDeltaMs;
  /** toggles rolling of object  on/off */
  bool turningR;
  /** toggles pitching of object on/off */
  bool turningP;
  /** toggles yawing of object on/off */
  bool turningY;

protected:
  virtual void configure(const map<string, string> &_config);
  virtual void start();
  virtual void destroy();
  virtual void runComponent();

public:
  GazeboTurntable();
};

}

#endif
