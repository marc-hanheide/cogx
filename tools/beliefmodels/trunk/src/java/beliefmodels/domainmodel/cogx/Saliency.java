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

public enum Saliency implements java.io.Serializable
{
    low,
    high;

    public static final int _low = 0;
    public static final int _high = 1;

    public static Saliency
    convert(int val)
    {
        assert val >= 0 && val < 2;
        return values()[val];
    }

    public static Saliency
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

    public static Saliency
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(2);
        return Saliency.convert(__v);
    }
}
