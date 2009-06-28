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

public interface ComponentFactoryPrx extends Ice.ObjectPrx
{
    public CASTComponentPrx newComponent(String id, String type)
        throws cast.ComponentCreationException;
    public CASTComponentPrx newComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException;

    public ManagedComponentPrx newManagedComponent(String id, String type)
        throws cast.ComponentCreationException;
    public ManagedComponentPrx newManagedComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException;

    public UnmanagedComponentPrx newUnmanagedComponent(String id, String type)
        throws cast.ComponentCreationException;
    public UnmanagedComponentPrx newUnmanagedComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException;

    public WorkingMemoryPrx newWorkingMemory(String id, String type)
        throws cast.ComponentCreationException;
    public WorkingMemoryPrx newWorkingMemory(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException;

    public TaskManagerPrx newTaskManager(String id, String type)
        throws cast.ComponentCreationException;
    public TaskManagerPrx newTaskManager(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException;
}
