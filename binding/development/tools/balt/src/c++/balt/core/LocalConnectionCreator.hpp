/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef LOCAL_CONNECTION_CREATOR
#define LOCAL_CONNECTION_CREATOR

#include "FrameworkProcess.hpp"
#include "FrameworkConnector.hpp"

class LocalConnectionCreator {

public:
  
  LocalConnectionCreator(){};
  virtual ~LocalConnectionCreator(){};

  virtual FrameworkConnector * createPushConnection(const std::vector<FrameworkProcess *> &_senders, 
						    const std::vector<FrameworkProcess *>  &_receivers, 
						    const std::string &  _connectorID) const = 0;
  
  virtual FrameworkConnector * createPullConnection(const std::vector<FrameworkProcess *> &_senders, 
						    FrameworkProcess * _receiver, 
						    const std::string &  _connectorID) const = 0;

};

#endif


//  LocalWords:  ifndef
