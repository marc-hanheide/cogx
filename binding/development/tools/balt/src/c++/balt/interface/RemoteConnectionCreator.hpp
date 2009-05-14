/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
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

#ifndef REMOTE_CONNECTION_CREATOR
#define REMOTE_CONNECTION_CREATOR

#include <balt/core/BALTCore.hpp>
#include <balt/idl/RemoteConnector.hh>
#include "includes.hpp"

class RemoteConnectionCreator {

public:

    RemoteConnectionCreator() {};
    virtual ~RemoteConnectionCreator() {};
    
    virtual 
    POA_RemoteConnectors::RemotePushSender * 
    connectAsSender(FrameworkProcess * _pSender, RemoteConnectors::RemotePushConnector_ptr rpc, const std::string & _connectionID) const = 0;

    virtual 
    POA_RemoteConnectors::RemotePushReceiver * 
    connectAsReceiver(FrameworkProcess * _pReceiver, RemoteConnectors::RemotePushConnector_ptr rpc, const std::string & _connectionID)  const = 0;

    virtual 
    POA_RemoteConnectors::RemotePullSender * 
    connectAsSender(FrameworkProcess * _pSender, RemoteConnectors::RemotePullConnector_ptr rpc, const std::string & _connectionID)  const = 0;

    virtual 
    POA_RemoteConnectors::RemotePullReceiver * 
    connectAsReceiver(FrameworkProcess * _pReceiver, RemoteConnectors::RemotePullConnector_ptr rpc, const std::string & _connectionID)  const = 0;

};

#endif

