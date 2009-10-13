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

public enum GroundStatus implements java.io.Serializable
{
    assertionVerified,
    assertionFalsified,
    propositionTrue,
    propositionFalse,
    propositionAmbiguous;

    public static final int _assertionVerified = 0;
    public static final int _assertionFalsified = 1;
    public static final int _propositionTrue = 2;
    public static final int _propositionFalse = 3;
    public static final int _propositionAmbiguous = 4;

    public static GroundStatus
    convert(int val)
    {
        assert val >= 0 && val < 5;
        return values()[val];
    }

    public static GroundStatus
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

    public static GroundStatus
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(5);
        return GroundStatus.convert(__v);
    }
}
