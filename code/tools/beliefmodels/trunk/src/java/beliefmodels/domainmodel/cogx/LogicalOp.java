// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.domainmodel.cogx;

public enum LogicalOp implements java.io.Serializable
{
    and,
    or,
    xor,
    none;

    public static final int _and = 0;
    public static final int _or = 1;
    public static final int _xor = 2;
    public static final int _none = 3;

    public static LogicalOp
    convert(int val)
    {
        assert val >= 0 && val < 4;
        return values()[val];
    }

    public static LogicalOp
    convert(String val)
    {
        try
        {
            return valueOf(val);
        }
        catch(java.lang.IllegalArgumentException ex)
        {
            return null;
        }
    }

    public int
    value()
    {
        return ordinal();
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeByte((byte)value());
    }

    public static LogicalOp
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(4);
        return LogicalOp.convert(__v);
    }
}
