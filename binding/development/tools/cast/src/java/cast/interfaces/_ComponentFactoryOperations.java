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

public interface _ComponentFactoryOperations
{
    CASTComponentPrx newComponent(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException;

    ManagedComponentPrx newManagedComponent(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException;

    UnmanagedComponentPrx newUnmanagedComponent(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException;

    WorkingMemoryPrx newWorkingMemory(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException;

    TaskManagerPrx newTaskManager(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException;
}
