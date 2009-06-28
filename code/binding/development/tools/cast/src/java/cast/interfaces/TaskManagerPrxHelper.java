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

public final class TaskManagerPrxHelper extends Ice.ObjectPrxHelperBase implements TaskManagerPrx
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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
    addManagedComponent(ManagedComponentPrx comp)
    {
        addManagedComponent(comp, null, false);
    }

    public void
    addManagedComponent(ManagedComponentPrx comp, java.util.Map<String, String> __ctx)
    {
        addManagedComponent(comp, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    addManagedComponent(ManagedComponentPrx comp, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
                __del.addManagedComponent(comp, __ctx);
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
    proposeTask(String component, String taskID, String taskName)
    {
        proposeTask(component, taskID, taskName, null, false);
    }

    public void
    proposeTask(String component, String taskID, String taskName, java.util.Map<String, String> __ctx)
    {
        proposeTask(component, taskID, taskName, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    proposeTask(String component, String taskID, String taskName, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
                __del.proposeTask(component, taskID, taskName, __ctx);
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
    retractTask(String component, String taskID)
    {
        retractTask(component, taskID, null, false);
    }

    public void
    retractTask(String component, String taskID, java.util.Map<String, String> __ctx)
    {
        retractTask(component, taskID, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    retractTask(String component, String taskID, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
                __del.retractTask(component, taskID, __ctx);
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
    taskComplete(String component, String taskID, cast.cdl.TaskOutcome outcome)
    {
        taskComplete(component, taskID, outcome, null, false);
    }

    public void
    taskComplete(String component, String taskID, cast.cdl.TaskOutcome outcome, java.util.Map<String, String> __ctx)
    {
        taskComplete(component, taskID, outcome, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    taskComplete(String component, String taskID, cast.cdl.TaskOutcome outcome, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
                __del.taskComplete(component, taskID, outcome, __ctx);
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
    setWorkingMemory(WorkingMemoryPrx wm)
    {
        setWorkingMemory(wm, null, false);
    }

    public void
    setWorkingMemory(WorkingMemoryPrx wm, java.util.Map<String, String> __ctx)
    {
        setWorkingMemory(wm, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setWorkingMemory(WorkingMemoryPrx wm, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
                __del.setWorkingMemory(wm, __ctx);
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
                _TaskManagerDel __del = (_TaskManagerDel)__delBase;
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

    public static TaskManagerPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TaskManagerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TaskManagerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::TaskManager"))
                {
                    TaskManagerPrxHelper __h = new TaskManagerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TaskManagerPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TaskManagerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TaskManagerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::TaskManager", __ctx))
                {
                    TaskManagerPrxHelper __h = new TaskManagerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TaskManagerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TaskManagerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::TaskManager"))
                {
                    TaskManagerPrxHelper __h = new TaskManagerPrxHelper();
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

    public static TaskManagerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TaskManagerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::TaskManager", __ctx))
                {
                    TaskManagerPrxHelper __h = new TaskManagerPrxHelper();
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

    public static TaskManagerPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TaskManagerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TaskManagerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TaskManagerPrxHelper __h = new TaskManagerPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TaskManagerPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TaskManagerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TaskManagerPrxHelper __h = new TaskManagerPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TaskManagerDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TaskManagerDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TaskManagerPrx v)
    {
        __os.writeProxy(v);
    }

    public static TaskManagerPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TaskManagerPrxHelper result = new TaskManagerPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
