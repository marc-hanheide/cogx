// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0

package cast;

public class ConsistencyException extends WMException
{
    public ConsistencyException()
    {
        super();
    }

    public ConsistencyException(String message, cast.cdl.WorkingMemoryAddress wma)
    {
        super(message, wma);
    }

    public String
    ice_name()
    {
        return "cast::ConsistencyException";
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeString("::cast::ConsistencyException");
        __os.startWriteSlice();
        __os.endWriteSlice();
        super.__write(__os);
    }

    public void
    __read(IceInternal.BasicStream __is, boolean __rid)
    {
        if(__rid)
        {
            __is.readString();
        }
        __is.startReadSlice();
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "exception cast::ConsistencyException was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "exception cast::ConsistencyException was not generated with stream support";
        throw ex;
    }
}
