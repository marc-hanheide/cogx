// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.interfaces;

public interface _CASTComponentOperations
{
    void beat(Ice.Current __current);

    void setID(String id, Ice.Current __current);

    String getID(Ice.Current __current);

    void configure(java.util.Map<java.lang.String, java.lang.String> config, Ice.Current __current);

    void start(Ice.Current __current);

    void run(Ice.Current __current);

    void stop(Ice.Current __current);

    void setComponentManager(ComponentManagerPrx man, Ice.Current __current);

    void setTimeServer(TimeServerPrx ts, Ice.Current __current);

    void destroy(Ice.Current __current);
}
