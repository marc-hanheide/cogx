/*
 * BALT - The Boxes and Lines Toolkit for component communication.
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

#ifndef LOCAL_DATATYPE_MANAGER
#define LOCAL_DATATYPE_MANAGER



#include "LocalConnectionCreator.hpp"
#include "includes.hpp"

typedef cast::StringMap<LocalConnectionCreator*>::map ConnectionCreatorMap;




/**
 * This class manages the creation of local connections (i.e. ones in
 * the same language on the same machine) for particular
 * datatypes. Initially this class supports many primitive datatypes
 *
 */
class LocalDatatypeManager {

 public:
  LocalDatatypeManager();
  ~LocalDatatypeManager();

  static void addDatatype(const std::string &_datatype, 
			  LocalConnectionCreator * _pCreator);

  static const LocalConnectionCreator * connectionCreator(const std::string &_datatype);

 private:
  static void init();
  static void builtInTypes();
  static ConnectionCreatorMap * m_pCreators;
  static bool m_bInit;


};


#endif
