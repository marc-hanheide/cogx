// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0

package cast.interfaces;

public interface _CASTComponentOperationsNC
{
    void beat();

    void setID(String id);

    String getID();

    void configure(java.util.Map<java.lang.String, java.lang.String> config);

    void start();

    void run();

    void stop();

    void setComponentManager(ComponentManagerPrx man);

    void setTimeServer(TimeServerPrx ts);

    void destroy();
}
