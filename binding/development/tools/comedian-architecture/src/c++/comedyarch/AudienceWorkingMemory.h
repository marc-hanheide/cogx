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

#ifndef CAST_AUDIENCE_WORKING_MEMORY_H_
#define CAST_AUDIENCE_WORKING_MEMORY_H_

#include <cast/architecture/SubarchitectureWorkingMemory.hpp>



class AudienceWorkingMemory : public cast::SubarchitectureWorkingMemory {

 public:
  AudienceWorkingMemory(const std::string & _id);
  ~AudienceWorkingMemory(){};
  
  virtual 
  void receivePushData(FrameworkLocalData<cast::CASTWorkingMemoryEntry> *_pData);

  virtual 
  void receivePushData(FrameworkLocalData<cast::cdl::WorkingMemoryEntry> *_pData);
  


};

#endif
