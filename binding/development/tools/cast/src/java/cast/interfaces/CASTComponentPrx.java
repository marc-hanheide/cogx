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

public interface CASTComponentPrx extends Ice.ObjectPrx
{
    public void beat();
    public void beat(java.util.Map<String, String> __ctx);

    public void setID(String id);
    public void setID(String id, java.util.Map<String, String> __ctx);

    public String getID();
    public String getID(java.util.Map<String, String> __ctx);

    public void configure(java.util.Map<java.lang.String, java.lang.String> config);
    public void configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx);

    public void start();
    public void start(java.util.Map<String, String> __ctx);

    public void run();
    public void run(java.util.Map<String, String> __ctx);

    public void stop();
    public void stop(java.util.Map<String, String> __ctx);

    public void setComponentManager(ComponentManagerPrx man);
    public void setComponentManager(ComponentManagerPrx man, java.util.Map<String, String> __ctx);

    public void setTimeServer(TimeServerPrx ts);
    public void setTimeServer(TimeServerPrx ts, java.util.Map<String, String> __ctx);

    public void destroy();
    public void destroy(java.util.Map<String, String> __ctx);
}
