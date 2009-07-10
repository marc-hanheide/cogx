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

public interface _ComponentFactoryDel extends Ice._ObjectDel
{
    CASTComponentPrx newComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ComponentCreationException;

    ManagedComponentPrx newManagedComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ComponentCreationException;

    UnmanagedComponentPrx newUnmanagedComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ComponentCreationException;

    WorkingMemoryPrx newWorkingMemory(String id, String type, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ComponentCreationException;

    TaskManagerPrx newTaskManager(String id, String type, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ComponentCreationException;
}
