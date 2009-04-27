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

#include "SubarchitectureComponent.hpp"

using namespace std;

namespace cast {

//   SubarchitectureComponent::SubarchitectureComponent(const string &_) : 
//     CASTComponent(_id) {
//   }


  void SubarchitectureComponent::configureInternal(const map<string,string> & _config) {

    //cout<<"SubarchitectureComponent::configure: "<<_config<<endl;
  
    CASTComponent::configureInternal(_config);

    map<string,string>::const_iterator i = _config.find(cdl::SUBARCHIDKEY);

    assert(i != _config.end());
    
    m_subarchitectureID = i->second;
  
  }



} //namespace cast
