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

public abstract class _TaskManagerDisp extends Ice.ObjectImpl implements TaskManager
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
        "::cast::interfaces::TaskManager",
        "::cast::interfaces::WorkingMemoryAttachedComponent",
        "::cast::interfaces::WorkingMemoryReaderComponent"
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
    addManagedComponent(ManagedComponentPrx comp)
    {
        addManagedComponent(comp, null);
    }

    public final void
    proposeTask(String component, String taskID, String taskName)
    {
        proposeTask(component, taskID, taskName, null);
    }

    public final void
    retractTask(String component, String taskID)
    {
        retractTask(component, taskID, null);
    }

    public final void
    taskComplete(String component, String taskID, cast.cdl.TaskOutcome outcome)
    {
        taskComplete(component, taskID, outcome, null);
    }

    public final void
    setWorkingMemory(WorkingMemoryPrx wm)
    {
        setWorkingMemory(wm, null);
    }

    public final void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc)
    {
        receiveChangeEvent(wmc, null);
    }

    public static Ice.DispatchStatus
    ___proposeTask(TaskManager __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String component;
        component = __is.readString();
        String taskID;
        taskID = __is.readString();
        String taskName;
        taskName = __is.readString();
        __is.endReadEncaps();
        __obj.proposeTask(component, taskID, taskName, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___retractTask(TaskManager __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String component;
        component = __is.readString();
        String taskID;
        taskID = __is.readString();
        __is.endReadEncaps();
        __obj.retractTask(component, taskID, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___taskComplete(TaskManager __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String component;
        component = __is.readString();
        String taskID;
        taskID = __is.readString();
        cast.cdl.TaskOutcome outcome;
        outcome = cast.cdl.TaskOutcome.__read(__is);
        __is.endReadEncaps();
        __obj.taskComplete(component, taskID, outcome, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    public static Ice.DispatchStatus
    ___addManagedComponent(TaskManager __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        ManagedComponentPrx comp;
        comp = ManagedComponentPrxHelper.__read(__is);
        __is.endReadEncaps();
        __obj.addManagedComponent(comp, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    private final static String[] __all =
    {
        "addManagedComponent",
        "beat",
        "configure",
        "destroy",
        "getID",
        "ice_id",
        "ice_ids",
        "ice_isA",
        "ice_ping",
        "proposeTask",
        "receiveChangeEvent",
        "retractTask",
        "run",
        "setComponentManager",
        "setID",
        "setTimeServer",
        "setWorkingMemory",
        "start",
        "stop",
        "taskComplete"
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
                return ___addManagedComponent(this, in, __current);
            }
            case 1:
            {
                return _CASTComponentDisp.___beat(this, in, __current);
            }
            case 2:
            {
                return _CASTComponentDisp.___configure(this, in, __current);
            }
            case 3:
            {
                return _CASTComponentDisp.___destroy(this, in, __current);
            }
            case 4:
            {
                return _CASTComponentDisp.___getID(this, in, __current);
            }
            case 5:
            {
                return ___ice_id(this, in, __current);
            }
            case 6:
            {
                return ___ice_ids(this, in, __current);
            }
            case 7:
            {
                return ___ice_isA(this, in, __current);
            }
            case 8:
            {
                return ___ice_ping(this, in, __current);
            }
            case 9:
            {
                return ___proposeTask(this, in, __current);
            }
            case 10:
            {
                return _WorkingMemoryReaderComponentDisp.___receiveChangeEvent(this, in, __current);
            }
            case 11:
            {
                return ___retractTask(this, in, __current);
            }
            case 12:
            {
                return _CASTComponentDisp.___run(this, in, __current);
            }
            case 13:
            {
                return _CASTComponentDisp.___setComponentManager(this, in, __current);
            }
            case 14:
            {
                return _CASTComponentDisp.___setID(this, in, __current);
            }
            case 15:
            {
                return _CASTComponentDisp.___setTimeServer(this, in, __current);
            }
            case 16:
            {
                return _WorkingMemoryAttachedComponentDisp.___setWorkingMemory(this, in, __current);
            }
            case 17:
            {
                return _CASTComponentDisp.___start(this, in, __current);
            }
            case 18:
            {
                return _CASTComponentDisp.___stop(this, in, __current);
            }
            case 19:
            {
                return ___taskComplete(this, in, __current);
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
        ex.reason = "type cast::interfaces::TaskManager was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type cast::interfaces::TaskManager was not generated with stream support";
        throw ex;
    }
}
