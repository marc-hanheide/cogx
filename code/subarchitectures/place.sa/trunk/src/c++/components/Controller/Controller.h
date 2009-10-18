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
 * PlaceController class.
 * \file Controller.h
 * \author Andrzej Pronobis
 * \date 2008-09-02
 */

#ifndef __PLACE_CONTROLLER__
#define __PLACE_CONTROLLER__

#include <cast/architecture/ManagedComponent.hpp>
#include <PlaceData.hpp>


/**
 * Implements the Controller component.
 */
class PlaceController: public cast::ManagedComponent
{
public: // Component management

  /** Constructor. */
  PlaceController();

  /** Destructor. */
  virtual ~PlaceController();

  /** Handles component configuration. */
  virtual void configure(const std::map<std::string, std::string> &config);

  /** Main thread of the component. */
  virtual void runComponent();

  /** Invoked on start. */
  virtual void start();

  /** Invoked on stop. */
  virtual void stop();


private:

  /** New PlaceCommand added to wm. */
  void newPlaceCommand(const cast::cdl::WorkingMemoryChange &change);

  void sendDataProviderCommand(PlaceData::DataProviderCommandType cmd);
  void sendVisualProcessorCommand(PlaceData::VisualProcessorCommandType cmd);
  void sendDataSaverCommand(PlaceData::DataSaverCommandType cmd);


private:

  /** Name of the config file group. */
  const std::string _cfgGroup;

  bool _start;

  enum UpdatesSentOn
  {
    VP_READ, DS_READ
  };

  UpdatesSentOn _updatesSentOn;
};


#endif

