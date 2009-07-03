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

#ifndef CAST_ASSISTANT_DIRECTOR_H_
#define CAST_ASSISTANT_DIRECTOR_H_

#include <cast/examples/comedyarch/ComedyEssentials.hpp>
#include <cast/architecture.hpp>




class AssistantDirector : 
  public cast::ManagedComponent {

public:
    virtual ~AssistantDirector(){};
  
  virtual void start();
  virtual void runComponent(){};

protected:
  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  
private:
  void handleTwoLiner(const cast::cdl::WorkingMemoryChange & _wmc);
  void handleReaction(const cast::cdl::WorkingMemoryChange & _wmc);

};


#endif
