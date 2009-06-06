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

public enum WorkingMemoryPermissions implements java.io.Serializable
{
    LOCKEDO,
    LOCKEDOD,
    LOCKEDODR,
    UNLOCKED,
    DOESNOTEXIST,
    ALREADYLOCKED;

    public static final int _LOCKEDO = 0;
    public static final int _LOCKEDOD = 1;
    public static final int _LOCKEDODR = 2;
    public static final int _UNLOCKED = 3;
    public static final int _DOESNOTEXIST = 4;
    public static final int _ALREADYLOCKED = 5;

    public static WorkingMemoryPermissions
    convert(int val)
    {
        assert val >= 0 && val < 6;
        return values()[val];
    }

    public static WorkingMemoryPermissions
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

    public static WorkingMemoryPermissions
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(6);
        return WorkingMemoryPermissions.convert(__v);
    }
}
