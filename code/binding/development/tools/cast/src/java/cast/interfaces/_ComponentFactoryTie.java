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

public class _ComponentFactoryTie extends _ComponentFactoryDisp implements Ice.TieBase
{
    public
    _ComponentFactoryTie()
    {
    }

    public
    _ComponentFactoryTie(_ComponentFactoryOperations delegate)
    {
        _ice_delegate = delegate;
    }

    public java.lang.Object
    ice_delegate()
    {
        return _ice_delegate;
    }

    public void
    ice_delegate(java.lang.Object delegate)
    {
        _ice_delegate = (_ComponentFactoryOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _ComponentFactoryTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_ComponentFactoryTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    public CASTComponentPrx
    newComponent(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException
    {
        return _ice_delegate.newComponent(id, type, __current);
    }

    public ManagedComponentPrx
    newManagedComponent(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException
    {
        return _ice_delegate.newManagedComponent(id, type, __current);
    }

    public TaskManagerPrx
    newTaskManager(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException
    {
        return _ice_delegate.newTaskManager(id, type, __current);
    }

    public UnmanagedComponentPrx
    newUnmanagedComponent(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException
    {
        return _ice_delegate.newUnmanagedComponent(id, type, __current);
    }

    public WorkingMemoryPrx
    newWorkingMemory(String id, String type, Ice.Current __current)
        throws cast.ComponentCreationException
    {
        return _ice_delegate.newWorkingMemory(id, type, __current);
    }

    private _ComponentFactoryOperations _ice_delegate;
}
