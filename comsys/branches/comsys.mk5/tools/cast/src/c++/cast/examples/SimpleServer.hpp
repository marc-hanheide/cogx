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

#ifndef SIMPLE_COMPONENT_HPP_
#define SIMPLE_COMPONENT_HPP_

#include <cast/core.hpp>

/**
 *  A simple class to demonstrate how to provide an Ice server in
 * C++. If you wish to use multiple inheritance in C++ your Ice
 * interface must inherit from cast::interfaces::CASTComponent. See
 * CDL.ice for an example. If you prefer to use aggregation see
 * SimpleAggregateServer instead.
 *
 * @author nah
 */
class SimpleServer : 
public cast::CASTComponent, 
public cast::examples::autogen::WordServerAsComponent {
  
public:
  
  /**
   * Empty destructor.
   */   
    virtual ~SimpleServer(){};
    
    virtual std::string getNewWord(const ::Ice::Current & _curr) {
        return "yikes";
    }

    
protected:
  virtual
  void
    start();

};

#endif
