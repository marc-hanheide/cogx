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

public enum WorkingMemoryLockRequest implements java.io.Serializable
{
    REQUESTLOCKO,
    REQUESTLOCKOD,
    REQUESTLOCKODR,
    REQUESTTRYLOCKO,
    REQUESTTRYLOCKOD,
    REQUESTTRYLOCKODR,
    REQUESTUNLOCK,
    REQUESTSTATUS;

    public static final int _REQUESTLOCKO = 0;
    public static final int _REQUESTLOCKOD = 1;
    public static final int _REQUESTLOCKODR = 2;
    public static final int _REQUESTTRYLOCKO = 3;
    public static final int _REQUESTTRYLOCKOD = 4;
    public static final int _REQUESTTRYLOCKODR = 5;
    public static final int _REQUESTUNLOCK = 6;
    public static final int _REQUESTSTATUS = 7;

    public static WorkingMemoryLockRequest
    convert(int val)
    {
        assert val >= 0 && val < 8;
        return values()[val];
    }

    public static WorkingMemoryLockRequest
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

    public static WorkingMemoryLockRequest
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(8);
        return WorkingMemoryLockRequest.convert(__v);
    }
}
