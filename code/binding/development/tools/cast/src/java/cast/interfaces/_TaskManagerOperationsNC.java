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

public interface _TaskManagerOperationsNC extends _WorkingMemoryReaderComponentOperationsNC
{
    void proposeTask(String component, String taskID, String taskName);

    void retractTask(String component, String taskID);

    void taskComplete(String component, String taskID, cast.cdl.TaskOutcome outcome);

    void addManagedComponent(ManagedComponentPrx comp);
}
