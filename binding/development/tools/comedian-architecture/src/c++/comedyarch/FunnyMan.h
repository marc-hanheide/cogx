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

#ifndef CAST_FUNNY_MAN_H_
#define CAST_FUNNY_MAN_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <cast/core/CASTDataCache.hpp>
#include "idl/ComedyEssentials.hh"

#include <vector>
#include <map>

typedef std::map < std::string, std::string > TypedDataMap; //easier this way!
typedef std::vector < std::string > TypedDataVector;

class FunnyMan : 
  public cast::ManagedProcess {

public:
  FunnyMan(const std::string &_id);
  virtual ~FunnyMan();
  
  virtual void runComponent();
  virtual void start();

protected:
  virtual void redrawGraphicsText();
  virtual void taskAdopted(const std::string &_taskID);
  virtual void taskRejected(const std::string &_taskID);
  
private:

  void newJokeAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void echo(const cast::cdl::WorkingMemoryChange & _wmc);

  void quip(const std::string &_id);
  std::string generatePunchline(const std::string &_setup);

  // Hashtable used to record the tasks we want to carry out
  TypedDataMap * m_pProposedProcessing;
  TypedDataVector * m_pSetups;

  //new addition: use a cache!
  cast::CASTDataCache<comedyarch::autogen::Joke> m_jokeCache;

};


#endif
