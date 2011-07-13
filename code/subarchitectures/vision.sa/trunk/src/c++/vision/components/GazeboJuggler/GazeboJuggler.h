/**
 * @author Marko Mahnič
 * @date July 2011
 *
 * Move objects inside a Gazebo simulation.
 *
 * A set of predefined objects can be moved between fixed locations.
 */

#ifndef GAZEBO_JUGGLER_H
#define GAZEBO_JUGGLER_H

#include <vector>
#include <string>
#include <libplayerc++/playerc++.h>
#include <cast/architecture/ManagedComponent.hpp>
#include "cogxmath.h"

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cogx
{

class GObject
{
public:
  std::string label;
  cogx::Math::Vector3 loc;
  cogx::Math::Vector3 pose; // x=roll, y=pitch, z=yaw
  GObject(const std::string& label)
  {
    this->label = label;
  }
};

class GazeboJuggler : public cast::ManagedComponent
{
private:
  /**
   * Name of robot model in gazebo world.
   */
  // static const std::string robotName;
  /**
   * host on which player server runs (typically localhost)
   */
  std::string m_playerHost;
  /**
   * player port number (typically 6665)
   */
  uint32_t m_playerPort;

  /** player client object */
  PlayerCc::PlayerClient *pRobot;
  /** interface to the simulation */
  PlayerCc::SimulationProxy *pSim;

  /**
   * list of objects we want to have detected
   */
  std::vector<GObject> m_objects;
  std::vector<cogx::Math::Vector3> m_locations;

#ifdef FEAT_VISUALIZATION
  class CDisplayClient: public cogx::display::CDisplayClient
  {
    GazeboJuggler* pJuggler;
  public:
    CDisplayClient() { pJuggler = NULL; }
    void setClientData(GazeboJuggler* pJuggler) { this->pJuggler = pJuggler; }
    //void handleEvent(const Visualization::TEvent &event); [>override<]
    void onDialogValueChanged(const std::string& dialogId, const std::string& name, const std::string& value);
    void handleDialogCommand(const std::string& dialogId, const std::string& command, const std::string& params);
  };
  CDisplayClient m_display;
#endif

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  virtual void destroy();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();
 
  void prepareObjects();

  // placeIndes out of range => remove object
  void moveObject(const std::string& label, int placeIndex);
public:
  GazeboJuggler();
};

}

#endif
// vim: set sw=2 ts=8 sts=4 et :vim
