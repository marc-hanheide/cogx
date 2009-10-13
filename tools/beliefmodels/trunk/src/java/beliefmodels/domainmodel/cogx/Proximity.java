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

public enum Proximity implements java.io.Serializable
{
    proximal,
    distal;

    public static final int _proximal = 0;
    public static final int _distal = 1;

    public static Proximity
    convert(int val)
    {
        assert val >= 0 && val < 2;
        return values()[val];
    }

    public static Proximity
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

    public static Proximity
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(2);
        return Proximity.convert(__v);
    }
}
