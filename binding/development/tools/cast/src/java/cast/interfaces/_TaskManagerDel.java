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

public interface _TaskManagerDel extends _WorkingMemoryReaderComponentDel
{
    void proposeTask(String component, String taskID, String taskName, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void retractTask(String component, String taskID, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void taskComplete(String component, String taskID, cast.cdl.TaskOutcome outcome, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void addManagedComponent(ManagedComponentPrx comp, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;
}
