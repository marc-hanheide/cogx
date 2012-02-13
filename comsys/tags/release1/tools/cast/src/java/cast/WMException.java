// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast;

public class WMException extends SubarchitectureComponentException
{
    public WMException()
    {
        super();
    }

    public WMException(String message, cast.cdl.WorkingMemoryAddress wma)
    {
        super(message);
        this.wma = wma;
    }

    public String
    ice_name()
    {
        return "cast::WMException";
    }

    public cast.cdl.WorkingMemoryAddress wma;

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeString("::cast::WMException");
        __os.startWriteSlice();
        wma.__write(__os);
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
        wma = new cast.cdl.WorkingMemoryAddress();
        wma.__read(__is);
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        __outS.writeString("::cast::WMException");
        __outS.startSlice();
        wma.ice_write(__outS);
        __outS.endSlice();
        super.__write(__outS);
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        if(__rid)
        {
            __inS.readString();
        }
        __inS.startSlice();
        wma = new cast.cdl.WorkingMemoryAddress();
        wma.ice_read(__inS);
        __inS.endSlice();
        super.__read(__inS, true);
    }
}