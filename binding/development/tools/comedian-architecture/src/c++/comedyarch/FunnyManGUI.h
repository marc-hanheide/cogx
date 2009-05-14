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
#include "idl/ComedyEssentials.hh"

#include <vector>
#include <map>
using namespace std;
using namespace cast;

typedef map < string, CASTData<comedyarch::autogen::Joke> *> TypedDataMap;
typedef vector < CASTData<comedyarch::autogen::Joke> *> TypedDataVector;

class FunnyMan : 
  public ManagedProcess {

public:
  FunnyMan(const string &_id);
  virtual ~FunnyMan();
  
  virtual void runComponent();

  virtual void start();

protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  
private:

  void newJokeAdded(const CAST::WorkingMemoryChange & _wmc);

  void quip(CASTData<comedyarch::autogen::Joke> *_pData);
  string generatePunchline(const string &_setup);

  // Hashtable used to record the tasks we want to carry out
  TypedDataMap * m_pProposedProcessing;
  TypedDataVector * m_pSetups;

};


#endif
