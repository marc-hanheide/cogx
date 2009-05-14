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

#ifndef FRAMEWORK_CONNECTOR
#define FRAMEWORK_CONNECTOR

#include "includes.hpp"

class FrameworkConnector {

public:

  /**
   * Connector constructor. Contructs the super class and stores the
   * connector ID.
   * 
   * @param _id A std::string that must uniquely identify this connector.
   */
  FrameworkConnector(const std::string & _id) :
    m_connectorID(_id)
  {

    //cout<<"start constructor: "<<_id<<endl;

    //cout<<"end constructor"<<endl;
    //cout.flush();
  }

  virtual ~FrameworkConnector() {

  }

  /**
   * Get the connector identifier of this connector.
   */
  const std::string & getConnectorIdentifier() const {
    return m_connectorID;
  }
  

  virtual void stopConnector(){};

private:
  /**
   * The ID of this connector. This is currently protected by this class
   * to ensure it remains unchanged after connector construction.
   */
  std::string m_connectorID;
  
};

#endif

