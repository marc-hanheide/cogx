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

public class UnknownSubarchitectureException extends SubarchitectureComponentException
{
    public UnknownSubarchitectureException()
    {
        super();
    }

    public UnknownSubarchitectureException(String message, String subarchitecture)
    {
        super(message);
        this.subarchitecture = subarchitecture;
    }

    public String
    ice_name()
    {
        return "cast::UnknownSubarchitectureException";
    }

    public String subarchitecture;

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeString("::cast::UnknownSubarchitectureException");
        __os.startWriteSlice();
        __os.writeString(subarchitecture);
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
        subarchitecture = __is.readString();
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        __outS.writeString("::cast::UnknownSubarchitectureException");
        __outS.startSlice();
        __outS.writeString(subarchitecture);
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
        subarchitecture = __inS.readString();
        __inS.endSlice();
        super.__read(__inS, true);
    }
}
