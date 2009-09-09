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

#include <cast/core.hpp>
#include <cast/architecture.hpp>
#include <cast/examples/comedyarch/ComedyEssentials.hpp>

class FunnyMan : 
  public cast::ManagedComponent {

public:
  virtual ~FunnyMan() {}
  

  virtual void start();
  virtual void configure(const std::map<std::string,std::string> & _config);

protected:

  virtual void runComponent();
  
private:

  void newTwoLinerAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void newOneLinerAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void newJokeAdded(const cast::cdl::WorkingMemoryChange & _wmc);

  void quip(const std::string &_id);
  const std::string & generatePunchline(const std::string &_setup);

  std::string m_punchline;


};


#endif
