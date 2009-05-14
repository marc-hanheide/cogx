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

#include "CASTProcessingComponent.hpp"
#include "CASTException.hpp"

using namespace std;

namespace cast {

CASTProcessingComponent::CASTProcessingComponent(const string &_id) : 
  //InspectableComponent(_id),
  CASTUIComponent(_id) {

}


void CASTProcessingComponent::configure(map<string,string> & _config) {

  //cout<<"CASTProcessingComponent::configure: "<<_config<<endl;
  
  CASTUIComponent::configure(_config);

  map<string,string>::const_iterator i = _config.find(cdl::SUBARCH_ID_KEY);

  if(i != _config.end()) {
    m_subarchitectureID = i->second;
  }
  else {
    println("Configure map missing property: " + string(cdl::SUBARCH_ID_KEY));
  }
}


cast::cdl::WorkingMemoryAddress 
CASTProcessingComponent::makeAdress(const std::string& _subarchID, 
				    const std::string& _id) {
  cast::cdl::WorkingMemoryAddress wma;
  wma.m_subarchitecture = CORBA::string_dup(_subarchID.c_str());
  wma.m_id = CORBA::string_dup(_id.c_str());
  return wma;
}

} //namespace cast
