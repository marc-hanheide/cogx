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

#ifndef EXAMPLE_MANAGED_COMPONENT_HPP_
#define EXAMPLE_MANAGED_COMPONENT_HPP_

#include <ManagedComponent.hpp>

/**
 * An abstract class to represent a component in a subarchitecture that
 * can read from a working memory and does have its operations
 * controlled by a task manager.
 * 
 * @author nah
 */
class ExampleComponent : 
  public cast::CASTComponent {
  
public:
  
  /**
   * Empty destructor.
   */
  virtual ~ExampleComponent(){};

protected:
  virtual
  void
  runComponent();

};

#endif
