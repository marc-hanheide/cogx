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

public enum Color implements java.io.Serializable
{
    red,
    blue,
    yellow,
    green,
    unknownColor;

    public static final int _red = 0;
    public static final int _blue = 1;
    public static final int _yellow = 2;
    public static final int _green = 3;
    public static final int _unknownColor = 4;

    public static Color
    convert(int val)
    {
        assert val >= 0 && val < 5;
        return values()[val];
    }

    public static Color
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

    public static Color
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(5);
        return Color.convert(__v);
    }
}
