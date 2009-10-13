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

public enum Shape implements java.io.Serializable
{
    cylindrical,
    spherical,
    cubic,
    unknownShape;

    public static final int _cylindrical = 0;
    public static final int _spherical = 1;
    public static final int _cubic = 2;
    public static final int _unknownShape = 3;

    public static Shape
    convert(int val)
    {
        assert val >= 0 && val < 4;
        return values()[val];
    }

    public static Shape
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

    public static Shape
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(4);
        return Shape.convert(__v);
    }
}
