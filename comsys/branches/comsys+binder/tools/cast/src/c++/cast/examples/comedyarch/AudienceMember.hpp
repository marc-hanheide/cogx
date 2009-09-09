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

#ifndef CAST_AUDIENCE_MEMBER_H_
#define CAST_AUDIENCE_MEMBER_H_

#include <cast/core.hpp>
#include <cast/architecture.hpp>
#include <cast/examples/comedyarch/ComedyEssentials.hpp>

#include <map>



typedef cast::StringMap<boost::shared_ptr< cast::CASTData<comedyarch::autogen::TwoLiner> > >::map TypedDataMap;

class AudienceMember : public cast::ManagedComponent {

public:
  virtual ~AudienceMember(){};
  
  virtual void runComponent();

  virtual void configure(const std::map<std::string,std::string> & _config);
  
  virtual void start();

private:
  void newTwoLinerAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  
  std::string generateAudienceReaction(const comedyarch::autogen::TwoLiner & _joke);

  // Hashtable used to record the tasks we want to carry out
  TypedDataMap * m_pProposedComponenting;

  std::string m_defaultReaction;

};


#endif
