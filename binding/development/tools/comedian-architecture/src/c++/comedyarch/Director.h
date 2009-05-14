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

#ifndef CAST_DIRECTOR_H_
#define CAST_DIRECTOR_H_

#include <cast/architecture/PrivilegedManagedProcess.hpp>
#include "idl/ComedyEssentials.hh"


class Director :
  public cast::PrivilegedManagedProcess {
  
public:
  Director(const std::string &_id);
  virtual ~Director();
  
  virtual void start();
  virtual void runComponent(){};
  virtual void configure(std::map<std::string,std::string> & _config);

protected:
  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  
private:
  void takeAction(const cast::cdl::WorkingMemoryChange & _actionChange);
  void askTheAudience(const cast::cdl::WorkingMemoryAddress & _jokeAddress);
  void checkTheReaction(const cast::cdl::WorkingMemoryAddress & _reactionAddress);
  bool m_testing;

};


#endif
