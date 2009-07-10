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

public interface _ManagedComponentOperations extends _WorkingMemoryReaderComponentOperations
{
    void setTaskManager(TaskManagerPrx tm, Ice.Current __current);

    void taskDecision(String id, cast.cdl.TaskManagementDecision decision, Ice.Current __current);
}
