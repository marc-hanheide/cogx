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

public interface _CASTComponentDel extends Ice._ObjectDel
{
    void beat(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void setID(String id, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    String getID(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void start(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void run(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void stop(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void setComponentManager(ComponentManagerPrx man, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void setTimeServer(TimeServerPrx ts, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void destroy(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;
}
