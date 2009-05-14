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

#ifndef CAST_CAST_PROCESSING_COMPONENT_H_
#define CAST_CAST_PROCESSING_COMPONENT_H_

#include  "CASTUIComponent.hpp"


namespace cast {

class CASTProcessingComponent : public CASTUIComponent {
 
 public:

  /**
   * Construct a new processing component with the given unique ID.
   * 
   * @param _id
   *            The id used to identify this component. 
   */
  CASTProcessingComponent(const std::string &_id);
  
  /**
   * Empty virtual destructor.
   */
  virtual ~CASTProcessingComponent(){};


  /**
   * Overrides the configure method from FrameworkProcess to use
   * _config to set the subarchitecture ID.
   * 
   * @param _config
   *            The ID of the subarchitecture which contains this
   *            component.
   */
  virtual void configure(std::map<std::string,std::string> & _config);

 protected:

protected:
  //the id of the subarchitecture that contains this component
  std::string m_subarchitectureID;

public:
  const std::string& subarchitectureID() const {return m_subarchitectureID;}

  ///to match java
  const std::string& getSubarchitectureID() const {return m_subarchitectureID;}

  
  /// convenience function for making adresses from strings
  static cast::cdl::WorkingMemoryAddress makeAdress(const std::string& _subarchID, 
						    const std::string& _id);

};

} //namespace cast

#endif
