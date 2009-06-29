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

public interface ManagedComponentPrx extends WorkingMemoryReaderComponentPrx
{
    public void setTaskManager(TaskManagerPrx tm);
    public void setTaskManager(TaskManagerPrx tm, java.util.Map<String, String> __ctx);

    public void taskDecision(String id, cast.cdl.TaskManagementDecision decision);
    public void taskDecision(String id, cast.cdl.TaskManagementDecision decision, java.util.Map<String, String> __ctx);
}
