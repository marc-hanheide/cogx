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

public abstract class _WorkingMemoryAttachedComponentDisp extends Ice.ObjectImpl implements WorkingMemoryAttachedComponent
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
        "::cast::interfaces::WorkingMemoryAttachedComponent"
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
    setWorkingMemory(WorkingMemoryPrx wm)
    {
        setWorkingMemory(wm, null);
    }

    public static Ice.DispatchStatus
    ___setWorkingMemory(WorkingMemoryAttachedComponent __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        WorkingMemoryPrx wm;
        wm = WorkingMemoryPrxHelper.__read(__is);
        __is.endReadEncaps();
        __obj.setWorkingMemory(wm, __current);
        return Ice.DispatchStatus.DispatchOK;
    }

    private final static String[] __all =
    {
        "beat",
        "configure",
        "destroy",
        "getID",
        "ice_id",
        "ice_ids",
        "ice_isA",
        "ice_ping",
        "run",
        "setComponentManager",
        "setID",
        "setTimeServer",
        "setWorkingMemory",
        "start",
        "stop"
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
                return _CASTComponentDisp.___beat(this, in, __current);
            }
            case 1:
            {
                return _CASTComponentDisp.___configure(this, in, __current);
            }
            case 2:
            {
                return _CASTComponentDisp.___destroy(this, in, __current);
            }
            case 3:
            {
                return _CASTComponentDisp.___getID(this, in, __current);
            }
            case 4:
            {
                return ___ice_id(this, in, __current);
            }
            case 5:
            {
                return ___ice_ids(this, in, __current);
            }
            case 6:
            {
                return ___ice_isA(this, in, __current);
            }
            case 7:
            {
                return ___ice_ping(this, in, __current);
            }
            case 8:
            {
                return _CASTComponentDisp.___run(this, in, __current);
            }
            case 9:
            {
                return _CASTComponentDisp.___setComponentManager(this, in, __current);
            }
            case 10:
            {
                return _CASTComponentDisp.___setID(this, in, __current);
            }
            case 11:
            {
                return _CASTComponentDisp.___setTimeServer(this, in, __current);
            }
            case 12:
            {
                return ___setWorkingMemory(this, in, __current);
            }
            case 13:
            {
                return _CASTComponentDisp.___start(this, in, __current);
            }
            case 14:
            {
                return _CASTComponentDisp.___stop(this, in, __current);
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
        ex.reason = "type cast::interfaces::WorkingMemoryAttachedComponent was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type cast::interfaces::WorkingMemoryAttachedComponent was not generated with stream support";
        throw ex;
    }
}
