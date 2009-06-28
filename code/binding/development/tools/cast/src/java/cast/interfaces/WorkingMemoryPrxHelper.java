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

public final class WorkingMemoryPrxHelper extends Ice.ObjectPrxHelperBase implements WorkingMemoryPrx
{
    public void
    beat()
    {
        beat(null, false);
    }

    public void
    beat(java.util.Map<String, String> __ctx)
    {
        beat(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    beat(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.beat(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    configure(java.util.Map<java.lang.String, java.lang.String> config)
    {
        configure(config, null, false);
    }

    public void
    configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx)
    {
        configure(config, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.configure(config, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    destroy()
    {
        destroy(null, false);
    }

    public void
    destroy(java.util.Map<String, String> __ctx)
    {
        destroy(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    destroy(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.destroy(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public String
    getID()
    {
        return getID(null, false);
    }

    public String
    getID(java.util.Map<String, String> __ctx)
    {
        return getID(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private String
    getID(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("getID");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                return __del.getID(__ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    run()
    {
        run(null, false);
    }

    public void
    run(java.util.Map<String, String> __ctx)
    {
        run(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    run(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.run(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    setComponentManager(ComponentManagerPrx man)
    {
        setComponentManager(man, null, false);
    }

    public void
    setComponentManager(ComponentManagerPrx man, java.util.Map<String, String> __ctx)
    {
        setComponentManager(man, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setComponentManager(ComponentManagerPrx man, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.setComponentManager(man, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    setID(String id)
    {
        setID(id, null, false);
    }

    public void
    setID(String id, java.util.Map<String, String> __ctx)
    {
        setID(id, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setID(String id, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.setID(id, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    setTimeServer(TimeServerPrx ts)
    {
        setTimeServer(ts, null, false);
    }

    public void
    setTimeServer(TimeServerPrx ts, java.util.Map<String, String> __ctx)
    {
        setTimeServer(ts, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setTimeServer(TimeServerPrx ts, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.setTimeServer(ts, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    start()
    {
        start(null, false);
    }

    public void
    start(java.util.Map<String, String> __ctx)
    {
        start(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    start(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.start(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    stop()
    {
        stop(null, false);
    }

    public void
    stop(java.util.Map<String, String> __ctx)
    {
        stop(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    stop(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.stop(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    addReader(WorkingMemoryReaderComponentPrx reader)
    {
        addReader(reader, null, false);
    }

    public void
    addReader(WorkingMemoryReaderComponentPrx reader, java.util.Map<String, String> __ctx)
    {
        addReader(reader, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    addReader(WorkingMemoryReaderComponentPrx reader, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.addReader(reader, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException
    {
        addToWorkingMemory(id, subarch, type, component, entry, null, false);
    }

    public void
    addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException
    {
        addToWorkingMemory(id, subarch, type, component, entry, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("addToWorkingMemory");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.addToWorkingMemory(id, subarch, type, component, entry, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    deleteFromWorkingMemory(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        deleteFromWorkingMemory(id, subarch, component, null, false);
    }

    public void
    deleteFromWorkingMemory(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        deleteFromWorkingMemory(id, subarch, component, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    deleteFromWorkingMemory(String id, String subarch, String component, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("deleteFromWorkingMemory");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.deleteFromWorkingMemory(id, subarch, component, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public boolean
    exists(String id, String subarch)
        throws cast.UnknownSubarchitectureException
    {
        return exists(id, subarch, null, false);
    }

    public boolean
    exists(String id, String subarch, java.util.Map<String, String> __ctx)
        throws cast.UnknownSubarchitectureException
    {
        return exists(id, subarch, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private boolean
    exists(String id, String subarch, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("exists");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                return __del.exists(id, subarch, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public cast.cdl.WorkingMemoryPermissions
    getPermissions(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getPermissions(id, subarch, null, false);
    }

    public cast.cdl.WorkingMemoryPermissions
    getPermissions(String id, String subarch, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getPermissions(id, subarch, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private cast.cdl.WorkingMemoryPermissions
    getPermissions(String id, String subarch, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("getPermissions");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                return __del.getPermissions(id, subarch, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public int
    getVersionNumber(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getVersionNumber(id, subarch, null, false);
    }

    public int
    getVersionNumber(String id, String subarch, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getVersionNumber(id, subarch, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private int
    getVersionNumber(String id, String subarch, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("getVersionNumber");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                return __del.getVersionNumber(id, subarch, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries)
        throws cast.UnknownSubarchitectureException
    {
        getWorkingMemoryEntries(type, subarch, count, component, entries, null, false);
    }

    public void
    getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, java.util.Map<String, String> __ctx)
        throws cast.UnknownSubarchitectureException
    {
        getWorkingMemoryEntries(type, subarch, count, component, entries, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("getWorkingMemoryEntries");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.getWorkingMemoryEntries(type, subarch, count, component, entries, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public cast.cdl.WorkingMemoryEntry
    getWorkingMemoryEntry(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getWorkingMemoryEntry(id, subarch, component, null, false);
    }

    public cast.cdl.WorkingMemoryEntry
    getWorkingMemoryEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return getWorkingMemoryEntry(id, subarch, component, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private cast.cdl.WorkingMemoryEntry
    getWorkingMemoryEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("getWorkingMemoryEntry");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                return __del.getWorkingMemoryEntry(id, subarch, component, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        lockEntry(id, subarch, component, permissions, null, false);
    }

    public void
    lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        lockEntry(id, subarch, component, permissions, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("lockEntry");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.lockEntry(id, subarch, component, permissions, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        overwriteWorkingMemory(id, subarch, type, component, entry, null, false);
    }

    public void
    overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        overwriteWorkingMemory(id, subarch, type, component, entry, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("overwriteWorkingMemory");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.overwriteWorkingMemory(id, subarch, type, component, entry, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc)
    {
        receiveChangeEvent(wmc, null, false);
    }

    public void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, java.util.Map<String, String> __ctx)
    {
        receiveChangeEvent(wmc, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.receiveChangeEvent(wmc, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter)
    {
        registerComponentFilter(filter, null, false);
    }

    public void
    registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
    {
        registerComponentFilter(filter, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.registerComponentFilter(filter, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch)
    {
        registerWorkingMemoryFilter(filter, subarch, null, false);
    }

    public void
    registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, java.util.Map<String, String> __ctx)
    {
        registerWorkingMemoryFilter(filter, subarch, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.registerWorkingMemoryFilter(filter, subarch, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter)
    {
        removeComponentFilter(filter, null, false);
    }

    public void
    removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
    {
        removeComponentFilter(filter, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.removeComponentFilter(filter, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter)
    {
        removeWorkingMemoryFilter(filter, null, false);
    }

    public void
    removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
    {
        removeWorkingMemoryFilter(filter, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.removeWorkingMemoryFilter(filter, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    setWorkingMemory(WorkingMemoryPrx wm, String subarch)
    {
        setWorkingMemory(wm, subarch, null, false);
    }

    public void
    setWorkingMemory(WorkingMemoryPrx wm, String subarch, java.util.Map<String, String> __ctx)
    {
        setWorkingMemory(wm, subarch, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setWorkingMemory(WorkingMemoryPrx wm, String subarch, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.setWorkingMemory(wm, subarch, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public boolean
    tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return tryLockEntry(id, subarch, component, permissions, null, false);
    }

    public boolean
    tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        return tryLockEntry(id, subarch, component, permissions, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private boolean
    tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("tryLockEntry");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                return __del.tryLockEntry(id, subarch, component, permissions, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    unlockEntry(String id, String subarch, String component)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        unlockEntry(id, subarch, component, null, false);
    }

    public void
    unlockEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        unlockEntry(id, subarch, component, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    unlockEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("unlockEntry");
                __delBase = __getDelegate(false);
                _WorkingMemoryDel __del = (_WorkingMemoryDel)__delBase;
                __del.unlockEntry(id, subarch, component, __ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public static WorkingMemoryPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        WorkingMemoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WorkingMemoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::WorkingMemory"))
                {
                    WorkingMemoryPrxHelper __h = new WorkingMemoryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WorkingMemoryPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        WorkingMemoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WorkingMemoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::WorkingMemory", __ctx))
                {
                    WorkingMemoryPrxHelper __h = new WorkingMemoryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WorkingMemoryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WorkingMemoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::WorkingMemory"))
                {
                    WorkingMemoryPrxHelper __h = new WorkingMemoryPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static WorkingMemoryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        WorkingMemoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::WorkingMemory", __ctx))
                {
                    WorkingMemoryPrxHelper __h = new WorkingMemoryPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static WorkingMemoryPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        WorkingMemoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WorkingMemoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                WorkingMemoryPrxHelper __h = new WorkingMemoryPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static WorkingMemoryPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WorkingMemoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            WorkingMemoryPrxHelper __h = new WorkingMemoryPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _WorkingMemoryDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _WorkingMemoryDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, WorkingMemoryPrx v)
    {
        __os.writeProxy(v);
    }

    public static WorkingMemoryPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            WorkingMemoryPrxHelper result = new WorkingMemoryPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
