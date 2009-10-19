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
 * \file PeekabotClient.h
 * \author Andrzej Pronobis
 * \date 2008-09-01
 */

#ifndef __PLACE_PEEKABOT_CLIENT__
#define __PLACE_PEEKABOT_CLIENT__

#include <PlaceData.hpp>
#include <cast/architecture/ManagedComponent.hpp>
// Peekabot
#include <peekabot.hh>
#include <peekabot/Types.hh>

/**
 * Implements the PeekabotClient component
 */
class PeekabotClient: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  PeekabotClient();

  /** Destructor. */
  virtual ~PeekabotClient();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string,std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  /** New VisualResults added to wm. */
  void newVisualResults(const cast::cdl::WorkingMemoryChange & change);

  /** New LaserResults added to wm. */
  void newLaserResults(const cast::cdl::WorkingMemoryChange & change);

  /** New IntegratedResults added to wm. */
  void newIntegratedResults(const cast::cdl::WorkingMemoryChange & change);

  /** Tries to connect to peekabot. */
  void connectPeekabot();

  void updateResults(const PlaceData::VisualResultsPtr,
      const PlaceData::LaserResultsPtr, const PlaceData::IntegratedResultsPtr);


private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  std::string _robotName;
  int _retryDelay;
  int _port;
  std::string _hostname;
  bool _loadScene;
  std::string _scene;

private:

  /** Queue for received visual results. */
  std::list<PlaceData::VisualResultsPtr> _visualQueue;

  /** Queue for received laser results. */
  std::list<PlaceData::LaserResultsPtr> _laserQueue;

  pthread_mutex_t _dataMutex;



  bool _connected;

  peekabot::PeekabotClient _client;
  peekabot::ObjectProxy _root;
  peekabot::GroupProxy _robot;
  peekabot::LabelProxy _laserLabel;
  peekabot::LabelProxy _visionLabel;
  peekabot::LabelProxy _intLabel;
  peekabot::CylinderProxy _laserCylinder;
  peekabot::CylinderProxy _visionCylinder;
  peekabot::CylinderProxy _intCylinder;
  peekabot::ModelProxy _laserScanner;
  peekabot::ModelProxy _camera;
  peekabot::CubeProxy _cube;
  peekabot::CubeProxy _plusIcon1;
  peekabot::CubeProxy _plusIcon2;
};


#endif

