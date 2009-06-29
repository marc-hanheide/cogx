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

public class CASTException extends Ice.UserException
{
    public CASTException()
    {
    }

    public CASTException(String message)
    {
        this.message = message;
    }

    public String
    ice_name()
    {
        return "cast::CASTException";
    }

    public String message;

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeString("::cast::CASTException");
        __os.startWriteSlice();
        __os.writeString(message);
        __os.endWriteSlice();
    }

    public void
    __read(IceInternal.BasicStream __is, boolean __rid)
    {
        if(__rid)
        {
            __is.readString();
        }
        __is.startReadSlice();
        message = __is.readString();
        __is.endReadSlice();
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        __outS.writeString("::cast::CASTException");
        __outS.startSlice();
        __outS.writeString(message);
        __outS.endSlice();
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        if(__rid)
        {
            __inS.readString();
        }
        __inS.startSlice();
        message = __inS.readString();
        __inS.endSlice();
    }
}
