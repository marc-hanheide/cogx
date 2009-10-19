// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * PlacePeekabotClient class.
 * \file PeekabotClient.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-01
 */

// Place.SA
#include "PeekabotClient.h"
#include "shared/ConfigFile.h"
#include "shared/ColorMap.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace cast;
using namespace place;
using boost::bad_lexical_cast;
using boost::lexical_cast;
using boost::shared_ptr;


// ------------------------------------------------------
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PeekabotClient();
  }
}


// ------------------------------------------------------
PeekabotClient::PeekabotClient(): _cfgGroup("PeekabotClient")
{
  _connected=false;

  pthread_mutex_init(&_dataMutex, 0);
}


// ------------------------------------------------------
PeekabotClient::~PeekabotClient()
{
  pthread_mutex_destroy(&_dataMutex);
}


// ------------------------------------------------------
void PeekabotClient::configure(const map<string,string> &config)
{
  // Read cmd line options
  map<string, string>::const_iterator it = config.find("--config");
  string configFile;
  if (it!=config.end())
    configFile = it->second;

  // Read config file
  ConfigFile cf;
  if (configFile.empty())
    println("No configuration file provided. Using defaults instead.");
  else
  {
    if (!cf.read(configFile))
      throw(CASTException(exceptionMessage(__HERE__,
          "Unable to load configuration file '%s'!", configFile.c_str() )));
  }

  // Get configuration from the file
  try
  {
    _robotName=cf.getStrValue(_cfgGroup, "RobotName", "Robone");
    _retryDelay=cf.getIntValue(_cfgGroup, "ConnectionRetryDelay", 1000000);
    _port=cf.getIntValue(_cfgGroup, "Port", 5050);
    _hostname=cf.getStrValue(_cfgGroup, "Hostname", "localhost");
    _loadScene=cf.getBoolValue(_cfgGroup, "LoadScene", false);
    _scene=cf.getStrValue(_cfgGroup, "Scene", "Robone.xml");
  }
  catch(bad_lexical_cast &)
  {
    throw(CASTException(exceptionMessage(__HERE__,
        "Incorrect item value in the config file '%s'!", configFile.c_str() )));
  }

  // Check values
  if (_retryDelay<1000)
  {
    throw(CASTException(exceptionMessage(__HERE__,
        "The value of ConnectionRetryDelay is too small!" )));
  }

  // Print the configuration
  log("Configuration:");
  log("-> Robot name: %s", _robotName.c_str());
  log("-> Connection retry delay: %d", _retryDelay);
  log("-> Peekabot hostname: %s", _hostname.c_str());
  log("-> Peekabot port: %d", _port);
  log("-> Load scene: %s", (_loadScene)?"yes":"no");
  log("-> Scene: %s", _scene.c_str());

}


// ------------------------------------------------------
void PeekabotClient::start()
{
  // wmChange
  MemberFunctionChangeReceiver<PeekabotClient> * pReceiver =
      new MemberFunctionChangeReceiver<PeekabotClient>(this, &PeekabotClient::newVisualResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::VisualResults>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PeekabotClient>(this, &PeekabotClient::newLaserResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::LaserResults>(cdl::OVERWRITE),
                   pReceiver );

  pReceiver =
      new MemberFunctionChangeReceiver<PeekabotClient>(this, &PeekabotClient::newIntegratedResults);
  addChangeFilter( createLocalTypeFilter<PlaceData::IntegratedResults>(cdl::OVERWRITE),
                   pReceiver );


  // Try to connect to peekabot
  connectPeekabot();

  debug("Started!");
}


// ------------------------------------------------------
void PeekabotClient::stop()
{
  // Disconnect from peekabot
  _client.disconnect();

  debug("Stop!");
}


// ------------------------------------------------------
void PeekabotClient::runComponent()
{
  debug("Running...");
  // set_priority(PRIORITY_HIGH); No counterpart in CAST 2.0

  // Wait until we are connected to peekabot
  while(!_client.is_connected())
  {
    usleep(_retryDelay);
    connectPeekabot();
  }

  // Do nothing
  while(isRunning())
  {
    usleep(100000);
  }

  debug("Completed!");
}


// ------------------------------------------------------
void PeekabotClient::newVisualResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Are we connected?
  if (!_connected)
    return;

  // Get
  shared_ptr<CASTData<PlaceData::VisualResults> > resCast =
      getWorkingMemoryEntry<PlaceData::VisualResults>(change.address);
  if (!resCast)
    return;
  PlaceData::VisualResultsPtr res=resCast->getData();

  // Addd to queue
  pthread_mutex_lock(&_dataMutex);
  _visualQueue.push_back(new PlaceData::VisualResults(*res));
  pthread_mutex_unlock(&_dataMutex);
}


// ------------------------------------------------------
void PeekabotClient::updateResults(const PlaceData::VisualResultsPtr visualResults,
    const PlaceData::LaserResultsPtr laserResults,
    const PlaceData::IntegratedResultsPtr integratedResults)
{
  // Laser
  float r,g,b;
  if (laserResults->status==PlaceData::DsValid)
  {
    if (laserResults->confident)
    {
      _laserLabel.set_text(laserResults->results[0].className);
      ColorMap::getColorForPlaceClass(laserResults->results[0].classNo, r,g,b);
      _laserCylinder.set_color(r,g,b);
    }
    else
    {
      //_laserLabel.set_text(string(PlaceData::UNKNOWN_CLASS_NAME));
      _laserLabel.set_text("");
      ColorMap::getColorForPlaceClass(PlaceData::UnknownClassNo, r,g,b);
      _laserCylinder.set_color(r,g,b);
    }
  }
  else
  {
    _laserLabel.set_text("");
    ColorMap::getColorForPlaceClass(0, r,g,b);
    _laserCylinder.set_color(r,g,b);
  }

  // Video
  if (visualResults->status==PlaceData::DsValid)
  {
    if (visualResults->confident)
    {
      _visionLabel.set_text(visualResults->results[0].className);
      ColorMap::getColorForPlaceClass(visualResults->results[0].classNo, r,g,b);
      _visionCylinder.set_color(r,g,b);
    }
    else
    {
      //_visionLabel.set_text(string(PlaceData::UNKNOWN_CLASS_NAME));
      _visionLabel.set_text("");
      ColorMap::getColorForPlaceClass(PlaceData::UnknownClassNo, r,g,b);
      _visionCylinder.set_color(r,g,b);
    }
  }
  else
  {
    _visionLabel.set_text("");
    ColorMap::getColorForPlaceClass(0, r,g,b);
    _visionCylinder.set_color(r,g,b);
  }

  // Integrated
  if (integratedResults->status==PlaceData::DsValid)
  {
    if (integratedResults->confident)
    {
      _intLabel.set_text(integratedResults->results[0].className);
      ColorMap::getColorForPlaceClass(integratedResults->results[0].classNo, r,g,b);
      _intCylinder.set_color(r,g,b);
    }
    else
    {
      //_intLabel.set_text(string(PlaceData::UNKNOWN_CLASS_NAME));
      _intLabel.set_text("");
      ColorMap::getColorForPlaceClass(PlaceData::UnknownClassNo, r,g,b);
      _intCylinder.set_color(r,g,b);
    }
  }
  else
  {
    _intLabel.set_text("");
    ColorMap::getColorForPlaceClass(0, r,g,b);
    _intCylinder.set_color(r,g,b);
  }
}


// ------------------------------------------------------
void PeekabotClient::newLaserResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Are we connected?
  if (!_connected)
    return;

  // Get
  shared_ptr<CASTData<PlaceData::LaserResults> > resCast =
      getWorkingMemoryEntry<PlaceData::LaserResults>(change.address);
  if (!resCast)
    return;
  PlaceData::LaserResultsPtr res=resCast->getData();

  // Addd to queue
  pthread_mutex_lock(&_dataMutex);
  _laserQueue.push_back(new PlaceData::LaserResults(*res));
  pthread_mutex_unlock(&_dataMutex);
}


// ------------------------------------------------------
void PeekabotClient::newIntegratedResults(const cast::cdl::WorkingMemoryChange & change)
{
  // Are we connected?
  if (!_connected)
    return;

  // Get
  shared_ptr<CASTData<PlaceData::IntegratedResults> > resCast =
      getWorkingMemoryEntry<PlaceData::IntegratedResults>(change.address);
  if (!resCast)
    return;
  PlaceData::IntegratedResultsPtr res=resCast->getData();

  // Get from queue
  pthread_mutex_lock(&_dataMutex);

  if ( (!_visualQueue.empty()) && (!_laserQueue.empty()) )
  {
    if (_visualQueue.front()->frameNo == _laserQueue.front()->frameNo)
    {
      PlaceData::LaserResultsPtr laserRes = _laserQueue.front();
      PlaceData::VisualResultsPtr visualRes = _visualQueue.front();
      _laserQueue.pop_front();
      _visualQueue.pop_front();

      updateResults(visualRes, laserRes, res);
    }
  }

  pthread_mutex_unlock(&_dataMutex);
}


// ------------------------------------------------------
void PeekabotClient::connectPeekabot()
{
  try 
  {
    // Connect
    debug("Trying to connect to Peekabot on host %s and port %d", _hostname.c_str(), _port);
    _client.connect(_hostname, _port, true);
    debug("Connection to Peekabot established!");

    // Assign root
    _root.assign(_client, "root");

    if (_loadScene)
    { // Add 
      _robot.add(_root, _robotName, peekabot::REPLACE_ON_CONFLICT);
      peekabot::Status s = _robot.load_scene(_scene).status();
      if (s.failed())
      {
        println("Could not load scene!");
        _client.disconnect();
        return;
      }
    }
    else
    { // Assign
      peekabot::Status s = _robot.assign(_root, _robotName).status();
      if (s.failed())
      {
        println("Could not assign to robot!");
        _client.disconnect();
        return;
      }
    }

    // Cube
    _cube.add(_robot, "placeCube", peekabot::REPLACE_ON_CONFLICT);
    _cube.set_position(0, -0.6, 0.8);
    _cube.set_rotation(-1.57, 0, 0);
    _cube.set_scale(0.7, 0.6, 0.03);
    _cube.set_color(0,0,0);
    _cube.set_opacity(0.2);

    // Icons
    _camera.add(_cube,"placeVisionIcon","sth_dcsg_var/cam_left.pbmf", peekabot::REPLACE_ON_CONFLICT);
    _camera.set_position(-0.25, 0.2, 0.06);
    _camera.set_rotation(0,0,0);
    _camera.set_scale(2,2,2);
    _camera.set_opacity(0.8, true);
    _laserScanner.add(_cube,"placeLaserIcon","sick_lms200.pbmf", peekabot::REPLACE_ON_CONFLICT);
    _laserScanner.set_position(-0.25, 0, 0.06);
    _laserScanner.set_rotation(0,0,0);
    _laserScanner.set_scale(0.7,0.7,0.7);
    _laserScanner.set_opacity(0.8, true);
    _plusIcon1.add(_cube,"placePlusIcon1",peekabot::REPLACE_ON_CONFLICT);
    _plusIcon1.set_position(-0.25, -0.2, 0.06);
    _plusIcon1.set_rotation(0,0,0);
    _plusIcon1.set_scale(0.15,0.05,0.05);
    _plusIcon1.set_opacity(0.8, true);
    _plusIcon1.set_color(0, 0.8, 0);
    _plusIcon2.add(_cube,"placePlusIcon2",peekabot::REPLACE_ON_CONFLICT);
    _plusIcon2.set_position(-0.25, -0.2, 0.06);
    _plusIcon2.set_rotation(1.57,0,0);
    _plusIcon2.set_scale(0.15,0.05,0.05);
    _plusIcon2.set_opacity(0.8, true);
    _plusIcon2.set_color(0, 0.8, 0);

    // Create cylinders
    float r,g,b;
    ColorMap::getColorForPlaceClass(0, r,g,b);
    _visionCylinder.add(_cube,"placeVisionCylinder",peekabot::REPLACE_ON_CONFLICT);
    _visionCylinder.set_position(0.1, 0.2, 0.06);
    _visionCylinder.set_rotation(0,1.57,0);
    _visionCylinder.set_scale(0.05,0.07,0.4);
    _visionCylinder.set_opacity(0.8, true);
    _visionCylinder.set_color(r, g, b);
    _laserCylinder.add(_cube,"placeLaserCylinder",peekabot::REPLACE_ON_CONFLICT);
    _laserCylinder.set_position(0.1, 0, 0.06);
    _laserCylinder.set_rotation(0,1.57,0);
    _laserCylinder.set_scale(0.05,0.07,0.4);
    _laserCylinder.set_opacity(0.8, true);
    _laserCylinder.set_color(r, g, b);
    _intCylinder.add(_cube,"placeIntCylinder",peekabot::REPLACE_ON_CONFLICT);
    _intCylinder.set_position(0.1, -0.2, 0.06);
    _intCylinder.set_rotation(0,1.57,0);
    _intCylinder.set_scale(0.05,0.07,0.4);
    _intCylinder.set_opacity(0.8, true);
    _intCylinder.set_color(r, g, b);

    // Create labels
    _visionLabel.add(_cube, "placeVisionLabel", peekabot::REPLACE_ON_CONFLICT);
    _visionLabel.set_text("");
    _visionLabel.set_position(0.1, 0.2-0.03, 0.11);
    _visionLabel.set_rotation(0,0,0);
    _visionLabel.set_scale(7, 7, 7);
    _visionLabel.set_alignment(peekabot::ALIGN_CENTER);
    _visionLabel.set_color(0,0,0);
    _visionLabel.set_opacity(1, true);

    _laserLabel.add(_cube, "placeLaserLabel", peekabot::REPLACE_ON_CONFLICT);
    _laserLabel.set_text("");
    _laserLabel.set_position(0.1, 0.0-0.03, 0.11);
    _laserLabel.set_rotation(0,0,0);
    _laserLabel.set_scale(7, 7, 7);
    _laserLabel.set_alignment(peekabot::ALIGN_CENTER); //see TextAlignment in peekabot/src/Types.hh for more.
    _laserLabel.set_color(0,0,0);
    _laserLabel.set_opacity(1, true);

    _intLabel.add(_cube, "placeIntLabel", peekabot::REPLACE_ON_CONFLICT);
    _intLabel.set_text("");
    _intLabel.set_position(0.1, -0.2-0.03, 0.11);
    _intLabel.set_rotation(0,0,0);
    _intLabel.set_scale(7, 7, 7);
    _intLabel.set_alignment(peekabot::ALIGN_CENTER); //see TextAlignment in peekabot/src/Types.hh for more.
    _intLabel.set_color(0,0,0);
    _intLabel.set_opacity(1, true);

    // Sync
    _client.sync();

    // Mark as connected
    _connected=true;
  }
  catch(std::exception &e) 
  {
    println("Caught exception when connecting to peekabot (%s).", e.what());
    _client.disconnect();
  }
}


