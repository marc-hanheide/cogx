// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.cdl;

public enum WorkingMemoryOperation implements java.io.Serializable
{
    ADD,
    OVERWRITE,
    DELETE,
    GET,
    WILDCARD;

    public static final int _ADD = 0;
    public static final int _OVERWRITE = 1;
    public static final int _DELETE = 2;
    public static final int _GET = 3;
    public static final int _WILDCARD = 4;

    public static WorkingMemoryOperation
    convert(int val)
    {
        assert val >= 0 && val < 5;
        return values()[val];
    }

    public static WorkingMemoryOperation
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

    public static WorkingMemoryOperation
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(5);
        return WorkingMemoryOperation.convert(__v);
    }
}
