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

#ifndef WORD_SERVER_CLIENT_HPP_
#define WORD_SERVER_CLIENT_HPP_



#include <cast/core.hpp>
#include <sstream>
#include <Ice/Ice.h>

using namespace cast;
using namespace cast::cdl;
using namespace cast::examples::autogen;


/**
 * A simple class to demonstrate how to access an Ice server in
 * C++. This assumes that the server was registered with the CAST
 * registerIceInterface method, you can look at it's implementation to
 * determine how to use the other methods available.
 * 
 * @author nah
 */
class WordServerClient : 
  public cast::CASTComponent {
  
public:
  
  /**
   * Empty destructor.
   */   
  virtual ~WordServerClient(){};
        
protected:


  virtual
  void
  runComponent();

};

#endif
