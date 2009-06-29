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

public class _WorkingMemoryTie extends _WorkingMemoryDisp implements Ice.TieBase
{
    public
    _WorkingMemoryTie()
    {
    }

    public
    _WorkingMemoryTie(_WorkingMemoryOperations delegate)
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
        _ice_delegate = (_WorkingMemoryOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _WorkingMemoryTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_WorkingMemoryTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    public void
    beat(Ice.Current __current)
    {
        _ice_delegate.beat(__current);
    }

    public void
    configure(java.util.Map<java.lang.String, java.lang.String> config, Ice.Current __current)
    {
        _ice_delegate.configure(config, __current);
    }

    public void
    destroy(Ice.Current __current)
    {
        _ice_delegate.destroy(__current);
    }

    public String
    getID(Ice.Current __current)
    {
        return _ice_delegate.getID(__current);
    }

    public void
    run(Ice.Current __current)
    {
        _ice_delegate.run(__current);
    }

    public void
    setComponentManager(ComponentManagerPrx man, Ice.Current __current)
    {
        _ice_delegate.setComponentManager(man, __current);
    }

    public void
    setID(String id, Ice.Current __current)
    {
        _ice_delegate.setID(id, __current);
    }

    public void
    setTimeServer(TimeServerPrx ts, Ice.Current __current)
    {
        _ice_delegate.setTimeServer(ts, __current);
    }

    public void
    start(Ice.Current __current)
    {
        _ice_delegate.start(__current);
    }

    public void
    stop(Ice.Current __current)
    {
        _ice_delegate.stop(__current);
    }

    public void
    addReader(WorkingMemoryReaderComponentPrx reader, Ice.Current __current)
    {
        _ice_delegate.addReader(reader, __current);
    }

    public void
    addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, Ice.Current __current)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException
    {
        _ice_delegate.addToWorkingMemory(id, subarch, type, component, entry, __current);
    }

    public void
    deleteFromWorkingMemory(String id, String subarch, String component, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        _ice_delegate.deleteFromWorkingMemory(id, subarch, component, __current);
    }

    public boolean
    exists(String id, String subarch, Ice.Current __current)
        throws cast.UnknownSubarchitectureException
    {
        return _ice_delegate.exists(id, subarch, __current);
    }

    public cast.cdl.WorkingMemoryPermissions
    getPermissions(String id, String subarch, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return _ice_delegate.getPermissions(id, subarch, __current);
    }

    public int
    getVersionNumber(String id, String subarch, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return _ice_delegate.getVersionNumber(id, subarch, __current);
    }

    public void
    getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, Ice.Current __current)
        throws cast.UnknownSubarchitectureException
    {
        _ice_delegate.getWorkingMemoryEntries(type, subarch, count, component, entries, __current);
    }

    public cast.cdl.WorkingMemoryEntry
    getWorkingMemoryEntry(String id, String subarch, String component, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return _ice_delegate.getWorkingMemoryEntry(id, subarch, component, __current);
    }

    public void
    lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        _ice_delegate.lockEntry(id, subarch, component, permissions, __current);
    }

    public void
    overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        _ice_delegate.overwriteWorkingMemory(id, subarch, type, component, entry, __current);
    }

    public void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, Ice.Current __current)
    {
        _ice_delegate.receiveChangeEvent(wmc, __current);
    }

    public void
    registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, Ice.Current __current)
    {
        _ice_delegate.registerComponentFilter(filter, __current);
    }

    public void
    registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, Ice.Current __current)
    {
        _ice_delegate.registerWorkingMemoryFilter(filter, subarch, __current);
    }

    public void
    removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, Ice.Current __current)
    {
        _ice_delegate.removeComponentFilter(filter, __current);
    }

    public void
    removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, Ice.Current __current)
    {
        _ice_delegate.removeWorkingMemoryFilter(filter, __current);
    }

    public void
    setWorkingMemory(WorkingMemoryPrx wm, String subarch, Ice.Current __current)
    {
        _ice_delegate.setWorkingMemory(wm, subarch, __current);
    }

    public boolean
    tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return _ice_delegate.tryLockEntry(id, subarch, component, permissions, __current);
    }

    public void
    unlockEntry(String id, String subarch, String component, Ice.Current __current)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        _ice_delegate.unlockEntry(id, subarch, component, __current);
    }

    private _WorkingMemoryOperations _ice_delegate;
}
