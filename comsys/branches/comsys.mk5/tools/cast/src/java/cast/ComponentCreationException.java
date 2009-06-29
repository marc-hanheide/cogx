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

public class ComponentCreationException extends CASTException
{
    public ComponentCreationException()
    {
        super();
    }

    public ComponentCreationException(String message)
    {
        super(message);
    }

    public String
    ice_name()
    {
        return "cast::ComponentCreationException";
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeString("::cast::ComponentCreationException");
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
        __outS.writeString("::cast::ComponentCreationException");
        __outS.startSlice();
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
        __inS.endSlice();
        super.__read(__inS, true);
    }
}
