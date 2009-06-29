/*
 * Comedian example code to demonstrate CAST functionality.
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

#include "StraightMan.hpp"
#include  <ComedyEssentials.hpp>

using namespace std;
using namespace cast;
using namespace comedyarch::autogen;

/**
 * This function is used to create an instance of this component when
 * the library is loaded.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new StraightMan();
  }
}


StraightMan::StraightMan() :
  m_setup("How many mice does it take to screw in a lightbulb?") {
}


void StraightMan::configure(const std::map<std::string,std::string> & _config) {

  std::map<std::string,std::string>::const_iterator configIter 
    = _config.find("--setup");
  
  if(configIter != _config.end()) {
    m_setup = configIter->second;
  }

  log("setup set: %s", m_setup.c_str());
}

const string & StraightMan::generateSetup() {
  return m_setup;
}

void StraightMan::runComponent() {

  while(isRunning()) {
    
    log("ahem");
      
//      timeval now;
//      gettimeofday(&now, NULL);
//      double doubleTime = now.tv_sec;
//      doubleTime += now.tv_usec / 1000000;
//      println("now time as double: %f", doubleTime);
//
//      const cast::interfaces::TimeServerPrx ts(getTimeServer());
//      
//      cdl::CASTTime ct(ts->fromTimeOfDayDouble(doubleTime));
//      
//      ostringstream outStream;
//            outStream<<"now time as a cast time: "<<ct;
//            println(outStream.str());
      
      
    // make up a joke
    TwoLinerPtr joke = new TwoLiner("",generateSetup());
    
    // and then make it available in the s-a working memory
    addToWorkingMemory(newDataID(),joke);        

    //sleep for some time
    sleepComponent(1000);      
    
  }
}
