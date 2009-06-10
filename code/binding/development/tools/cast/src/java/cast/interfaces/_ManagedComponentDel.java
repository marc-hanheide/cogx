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

public interface _ManagedComponentDel extends _WorkingMemoryReaderComponentDel
{
    void setTaskManager(TaskManagerPrx tm, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void taskDecision(String id, cast.cdl.TaskManagementDecision decision, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;
}
