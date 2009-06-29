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

public abstract class _WorkingMemoryDisp extends Ice.ObjectImpl implements WorkingMemory
{
    protected void
    ice_copyStateFrom(Ice.Object __obj)
        throws java.lang.CloneNotSupportedException
    {
        throw new java.lang.CloneNotSupportedException();
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::cast::interfaces::CASTComponent",
        "::cast::interfaces::WorkingMemory"
    };

    public boolean
    ice_isA(String s)
    {
        return java.util.Arrays.binarySearch(__ids, s) >= 0;
    }

    public boolean
    ice_isA(String s, Ice.Current __current)
    {
        return java.util.Arrays.binarySearch(__ids, s) >= 0;
    }

    public String[]
    ice_ids()
    {
        return __ids;
    }

    public String[]
    ice_ids(Ice.Current __current)
    {
        return __ids;
    }

    public String
    ice_id()
    {
        return __ids[2];
    }

    public String
    ice_id(Ice.Current __current)
    {
        return __ids[2];
    }

    public static String
    ice_staticId()
    {
        return __ids[2];
    }

    public final void
    beat()
    {
        beat(null);
    }

    public final void
    configure(java.util.Map<java.lang.String, java.lang.String> config)
    {
        configure(config, null);
    }

    public final void
    destroy()
    {
        destroy(null);
    }

    public final String
    getID()
    {
        return getID(null);
    }

    public final void
    run()
    {
        run(null);
    }

    public final void
    setComponentManager(ComponentManagerPrx man)
    {
        setComponentManager(man, null);
    }

    public final void
    setID(String id)
    {
        setID(id, null);
    }

    public final void
    setTimeServer(TimeServerPrx ts)
    {
        setTimeServer(ts, null);
    }

    public final void
    start()
    {
        start(null);
    }

    public final void
    stop()
    {
        stop(null);
    }

    public final void
    addReader(WorkingMemoryReaderComponentPrx reader)
    {
        addReader(reader, null);
    }

    public final void
    addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException
    {
        addToWorkingMemory(id, subarch, type, component, entry, null);
    }

    public final void
    deleteFromWorkingMemory(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        deleteFromWorkingMemory(id, subarch, component, null);
    }

    public final boolean
    exists(String id, String subarch)
        throws cast.UnknownSubarchitectureException
    {
        return exists(id, subarch, null);
    }

    public final cast.cdl.WorkingMemoryPermissions
    getPermissions(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getPermissions(id, subarch, null);
    }

    public final int
    getVersionNumber(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getVersionNumber(id, subarch, null);
    }

    public final void
    getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries)
        throws cast.UnknownSubarchitectureException
    {
        getWorkingMemoryEntries(type, subarch, count, component, entries, null);
    }

    public final cast.cdl.WorkingMemoryEntry
    getWorkingMemoryEntry(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getWorkingMemoryEntry(id, subarch, component, null);
    }

    public final void
    lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        lockEntry(id, subarch, component, permissions, null);
    }

    public final void
    overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        overwriteWorkingMemory(id, subarch, type, component, entry, null);
    }

    public final void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc)
    {
        receiveChangeEvent(wmc, null);
    }

    public final void
    registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter)
    {
        registerComponentFilter(filter, null);
    }

    public final void
    registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch)
    {
        registerWorkingMemoryFilter(filter, subarch, null);
    }

    public final void
    removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter)
    {
        removeComponentFilter(filter, null);
    }

    public final void
    removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter)
    {
        removeWorkingMemoryFilter(filter, null);
    }

    public final void
    setWorkingMemory(WorkingMemoryPrx wm, String subarch)
    {
        setWorkingMemory(wm, subarch, null);
    }

    public final boolean
    tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return tryLockEntry(id, subarch, component, permissions, null);
    }

    public final void
    unlockEntry(String id, String subarch, String component)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        unlockEntry(id, subarch, component, null);
    }

    public static Ice.DispatchStatus
    ___exists(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Idempotent, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            boolean __ret = __obj.exists(id, subarch, __current);
            __os.writeBool(__ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___getVersionNumber(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Idempotent, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            int __ret = __obj.getVersionNumber(id, subarch, __current);
            __os.writeInt(__ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___getPermissions(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Idempotent, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            cast.cdl.WorkingMemoryPermissions __ret = __obj.getPermissions(id, subarch, __current);
            __ret.__write(__os);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___lockEntry(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String component;
        component = __is.readString();
        cast.cdl.WorkingMemoryPermissions permissions;
        permissions = cast.cdl.WorkingMemoryPermissions.__read(__is);
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            __obj.lockEntry(id, subarch, component, permissions, __current);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___tryLockEntry(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String component;
        component = __is.readString();
        cast.cdl.WorkingMemoryPermissions permissions;
        permissions = cast.cdl.WorkingMemoryPermissions.__read(__is);
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            boolean __ret = __obj.tryLockEntry(id, subarch, component, permissions, __current);
            __os.writeBool(__ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___unlockEntry(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String component;
        component = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            __obj.unlockEntry(id, subarch, component, __current);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.ConsistencyException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___setWorkingMemory(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        WorkingMemoryPrx wm;
        wm = WorkingMemoryPrxHelper.__read(__is);
        String subarch;
        subarch = __is.readString();
        __is.endReadEncaps();
        __obj.setWorkingMemory(wm, subarch, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___addToWorkingMemory(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String type;
        type = __is.readString();
        String component;
        component = __is.readString();
        Ice.ObjectHolder entry = new Ice.ObjectHolder();
        __is.readObject(entry.getPatcher());
        __is.readPendingObjects();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            __obj.addToWorkingMemory(id, subarch, type, component, entry.value, __current);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.AlreadyExistsOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___overwriteWorkingMemory(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String type;
        type = __is.readString();
        String component;
        component = __is.readString();
        Ice.ObjectHolder entry = new Ice.ObjectHolder();
        __is.readObject(entry.getPatcher());
        __is.readPendingObjects();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            __obj.overwriteWorkingMemory(id, subarch, type, component, entry.value, __current);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___deleteFromWorkingMemory(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String component;
        component = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            __obj.deleteFromWorkingMemory(id, subarch, component, __current);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___getWorkingMemoryEntry(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String subarch;
        subarch = __is.readString();
        String component;
        component = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            cast.cdl.WorkingMemoryEntry __ret = __obj.getWorkingMemoryEntry(id, subarch, component, __current);
            __os.writeObject(__ret);
            __os.writePendingObjects();
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.DoesNotExistOnWMException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___getWorkingMemoryEntries(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String type;
        type = __is.readString();
        String subarch;
        subarch = __is.readString();
        int count;
        count = __is.readInt();
        String component;
        component = __is.readString();
        __is.endReadEncaps();
        cast.cdl.WorkingMemoryEntrySeqHolder entries = new cast.cdl.WorkingMemoryEntrySeqHolder();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            __obj.getWorkingMemoryEntries(type, subarch, count, component, entries, __current);
            cast.cdl.WorkingMemoryEntrySeqHelper.write(__os, entries.value);
            __os.writePendingObjects();
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.UnknownSubarchitectureException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___registerComponentFilter(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        cast.cdl.WorkingMemoryChangeFilter filter;
        filter = new cast.cdl.WorkingMemoryChangeFilter();
        filter.__read(__is);
        __is.endReadEncaps();
        __obj.registerComponentFilter(filter, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___removeComponentFilter(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        cast.cdl.WorkingMemoryChangeFilter filter;
        filter = new cast.cdl.WorkingMemoryChangeFilter();
        filter.__read(__is);
        __is.endReadEncaps();
        __obj.removeComponentFilter(filter, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___registerWorkingMemoryFilter(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        cast.cdl.WorkingMemoryChangeFilter filter;
        filter = new cast.cdl.WorkingMemoryChangeFilter();
        filter.__read(__is);
        String subarch;
        subarch = __is.readString();
        __is.endReadEncaps();
        __obj.registerWorkingMemoryFilter(filter, subarch, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___removeWorkingMemoryFilter(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        cast.cdl.WorkingMemoryChangeFilter filter;
        filter = new cast.cdl.WorkingMemoryChangeFilter();
        filter.__read(__is);
        __is.endReadEncaps();
        __obj.removeWorkingMemoryFilter(filter, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___addReader(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        WorkingMemoryReaderComponentPrx reader;
        reader = WorkingMemoryReaderComponentPrxHelper.__read(__is);
        __is.endReadEncaps();
        __obj.addReader(reader, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___receiveChangeEvent(WorkingMemory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        cast.cdl.WorkingMemoryChange wmc;
        wmc = new cast.cdl.WorkingMemoryChange();
        wmc.__read(__is);
        __is.endReadEncaps();
        __obj.receiveChangeEvent(wmc, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    private final static String[] __all =
    {
        "addReader",
        "addToWorkingMemory",
        "beat",
        "configure",
        "deleteFromWorkingMemory",
        "destroy",
        "exists",
        "getID",
        "getPermissions",
        "getVersionNumber",
        "getWorkingMemoryEntries",
        "getWorkingMemoryEntry",
        "ice_id",
        "ice_ids",
        "ice_isA",
        "ice_ping",
        "lockEntry",
        "overwriteWorkingMemory",
        "receiveChangeEvent",
        "registerComponentFilter",
        "registerWorkingMemoryFilter",
        "removeComponentFilter",
        "removeWorkingMemoryFilter",
        "run",
        "setComponentManager",
        "setID",
        "setTimeServer",
        "setWorkingMemory",
        "start",
        "stop",
        "tryLockEntry",
        "unlockEntry"
    };

    public Ice.DispatchStatus
    __dispatch(IceInternal.Incoming in, Ice.Current __current)
    {
        int pos = java.util.Arrays.binarySearch(__all, __current.operation);
        if(pos < 0)
        {
            throw new Ice.OperationNotExistException(__current.id, __current.facet, __current.operation);
        }

        switch(pos)
        {
            case 0:
            {
                return ___addReader(this, in, __current);
            }
            case 1:
            {
                return ___addToWorkingMemory(this, in, __current);
            }
            case 2:
            {
                return _CASTComponentDisp.___beat(this, in, __current);
            }
            case 3:
            {
                return _CASTComponentDisp.___configure(this, in, __current);
            }
            case 4:
            {
                return ___deleteFromWorkingMemory(this, in, __current);
            }
            case 5:
            {
                return _CASTComponentDisp.___destroy(this, in, __current);
            }
            case 6:
            {
                return ___exists(this, in, __current);
            }
            case 7:
            {
                return _CASTComponentDisp.___getID(this, in, __current);
            }
            case 8:
            {
                return ___getPermissions(this, in, __current);
            }
            case 9:
            {
                return ___getVersionNumber(this, in, __current);
            }
            case 10:
            {
                return ___getWorkingMemoryEntries(this, in, __current);
            }
            case 11:
            {
                return ___getWorkingMemoryEntry(this, in, __current);
            }
            case 12:
            {
                return ___ice_id(this, in, __current);
            }
            case 13:
            {
                return ___ice_ids(this, in, __current);
            }
            case 14:
            {
                return ___ice_isA(this, in, __current);
            }
            case 15:
            {
                return ___ice_ping(this, in, __current);
            }
            case 16:
            {
                return ___lockEntry(this, in, __current);
            }
            case 17:
            {
                return ___overwriteWorkingMemory(this, in, __current);
            }
            case 18:
            {
                return ___receiveChangeEvent(this, in, __current);
            }
            case 19:
            {
                return ___registerComponentFilter(this, in, __current);
            }
            case 20:
            {
                return ___registerWorkingMemoryFilter(this, in, __current);
            }
            case 21:
            {
                return ___removeComponentFilter(this, in, __current);
            }
            case 22:
            {
                return ___removeWorkingMemoryFilter(this, in, __current);
            }
            case 23:
            {
                return _CASTComponentDisp.___run(this, in, __current);
            }
            case 24:
            {
                return _CASTComponentDisp.___setComponentManager(this, in, __current);
            }
            case 25:
            {
                return _CASTComponentDisp.___setID(this, in, __current);
            }
            case 26:
            {
                return _CASTComponentDisp.___setTimeServer(this, in, __current);
            }
            case 27:
            {
                return ___setWorkingMemory(this, in, __current);
            }
            case 28:
            {
                return _CASTComponentDisp.___start(this, in, __current);
            }
            case 29:
            {
                return _CASTComponentDisp.___stop(this, in, __current);
            }
            case 30:
            {
                return ___tryLockEntry(this, in, __current);
            }
            case 31:
            {
                return ___unlockEntry(this, in, __current);
            }
        }

        assert(false);
        throw new Ice.OperationNotExistException(__current.id, __current.facet, __current.operation);
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeTypeId(ice_staticId());
        __os.startWriteSlice();
        __os.endWriteSlice();
        super.__write(__os);
    }

    public void
    __read(IceInternal.BasicStream __is, boolean __rid)
    {
        if(__rid)
        {
            __is.readTypeId();
        }
        __is.startReadSlice();
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type cast::interfaces::WorkingMemory was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type cast::interfaces::WorkingMemory was not generated with stream support";
        throw ex;
    }
}
