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

public enum ObjectType implements java.io.Serializable
{
    box,
    ball,
    cube,
    mug,
    unknownObjectType;

    public static final int _box = 0;
    public static final int _ball = 1;
    public static final int _cube = 2;
    public static final int _mug = 3;
    public static final int _unknownObjectType = 4;

    public static ObjectType
    convert(int val)
    {
        assert val >= 0 && val < 5;
        return values()[val];
    }

    public static ObjectType
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

    public static ObjectType
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(5);
        return ObjectType.convert(__v);
    }
}
