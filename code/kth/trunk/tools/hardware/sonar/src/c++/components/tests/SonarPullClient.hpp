/*
 * CAST - The CoSy Architecture Schema Toolkit
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

#ifndef SONAR_PULL_CLIENT_HPP_
#define SONAR_PULL_CLIENT_HPP_



#include <cast/core.hpp>
#include <sstream>
#include <Ice/Ice.h>
#include <Sonar.hpp>

using namespace cast;
using namespace cast::cdl;
using namespace cast::examples::autogen;


/**
 * Client for the sonar server.
 * 
 * @author nah
 */
class SonarPullClient : 
  public cast::CASTComponent {
  
public:

  SonarPullClient();

  
  /**
   * Empty destructor.
   */   
  virtual ~SonarPullClient(){};
        
protected:


  virtual
  void
  runComponent();

  virtual 
  void 
  configure(const std::map<std::string,std::string> & config);


private:

  ///component name of server from cast file
  std::string m_sonarServerName;

};

#endif
