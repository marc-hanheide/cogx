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

#ifndef CAST_STRAIGHT_MAN_H_
#define CAST_STRAIGHT_MAN_H_

#include <cast/core.hpp>
#include <cast/architecture.hpp>
#include <cast/examples/comedyarch/ComedyEssentials.hpp>



class StraightMan : public cast::UnmanagedComponent {

public:

  StraightMan();  
  virtual ~StraightMan() {}

  /**
   * Do something at runtime.
   **/
  virtual void runComponent();

  /**
   * Receive configuration information.
   */
  virtual void configure(const std::map<std::string,std::string> & _config);

 private:

  /**
   * Setup the straight-man feed line.
   **/
  const std::string & generateSetup();

  std::string m_setup;

};


#endif
