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

public abstract class _ComponentFactoryDisp extends Ice.ObjectImpl implements ComponentFactory
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
        "::cast::interfaces::ComponentFactory"
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
        return __ids[1];
    }

    public String
    ice_id(Ice.Current __current)
    {
        return __ids[1];
    }

    public static String
    ice_staticId()
    {
        return __ids[1];
    }

    public final CASTComponentPrx
    newComponent(String id, String type)
        throws cast.ComponentCreationException
    {
        return newComponent(id, type, null);
    }

    public final ManagedComponentPrx
    newManagedComponent(String id, String type)
        throws cast.ComponentCreationException
    {
        return newManagedComponent(id, type, null);
    }

    public final TaskManagerPrx
    newTaskManager(String id, String type)
        throws cast.ComponentCreationException
    {
        return newTaskManager(id, type, null);
    }

    public final UnmanagedComponentPrx
    newUnmanagedComponent(String id, String type)
        throws cast.ComponentCreationException
    {
        return newUnmanagedComponent(id, type, null);
    }

    public final WorkingMemoryPrx
    newWorkingMemory(String id, String type)
        throws cast.ComponentCreationException
    {
        return newWorkingMemory(id, type, null);
    }

    public static Ice.DispatchStatus
    ___newComponent(ComponentFactory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String type;
        type = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            CASTComponentPrx __ret = __obj.newComponent(id, type, __current);
            CASTComponentPrxHelper.__write(__os, __ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.ComponentCreationException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___newManagedComponent(ComponentFactory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String type;
        type = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            ManagedComponentPrx __ret = __obj.newManagedComponent(id, type, __current);
            ManagedComponentPrxHelper.__write(__os, __ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.ComponentCreationException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___newUnmanagedComponent(ComponentFactory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String type;
        type = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            UnmanagedComponentPrx __ret = __obj.newUnmanagedComponent(id, type, __current);
            UnmanagedComponentPrxHelper.__write(__os, __ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.ComponentCreationException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___newWorkingMemory(ComponentFactory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String type;
        type = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            WorkingMemoryPrx __ret = __obj.newWorkingMemory(id, type, __current);
            WorkingMemoryPrxHelper.__write(__os, __ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.ComponentCreationException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    public static Ice.DispatchStatus
    ___newTaskManager(ComponentFactory __obj, IceInternal.Incoming __inS, Ice.Current __current)
    {
        __checkMode(Ice.OperationMode.Normal, __current.mode);
        IceInternal.BasicStream __is = __inS.is();
        __is.startReadEncaps();
        String id;
        id = __is.readString();
        String type;
        type = __is.readString();
        __is.endReadEncaps();
        IceInternal.BasicStream __os = __inS.os();
        try
        {
            TaskManagerPrx __ret = __obj.newTaskManager(id, type, __current);
            TaskManagerPrxHelper.__write(__os, __ret);
            return Ice.DispatchStatus.DispatchOK;
        }
        catch(cast.ComponentCreationException ex)
        {
            __os.writeUserException(ex);
            return Ice.DispatchStatus.DispatchUserException;
        }
    }

    private final static String[] __all =
    {
        "ice_id",
        "ice_ids",
        "ice_isA",
        "ice_ping",
        "newComponent",
        "newManagedComponent",
        "newTaskManager",
        "newUnmanagedComponent",
        "newWorkingMemory"
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
                return ___ice_id(this, in, __current);
            }
            case 1:
            {
                return ___ice_ids(this, in, __current);
            }
            case 2:
            {
                return ___ice_isA(this, in, __current);
            }
            case 3:
            {
                return ___ice_ping(this, in, __current);
            }
            case 4:
            {
                return ___newComponent(this, in, __current);
            }
            case 5:
            {
                return ___newManagedComponent(this, in, __current);
            }
            case 6:
            {
                return ___newTaskManager(this, in, __current);
            }
            case 7:
            {
                return ___newUnmanagedComponent(this, in, __current);
            }
            case 8:
            {
                return ___newWorkingMemory(this, in, __current);
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
        ex.reason = "type cast::interfaces::ComponentFactory was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type cast::interfaces::ComponentFactory was not generated with stream support";
        throw ex;
    }
}
