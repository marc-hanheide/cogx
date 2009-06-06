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

public enum WorkingMemoryChangeQueueBehaviour implements java.io.Serializable
{
    DISCARD,
    QUEUE;

    public static final int _DISCARD = 0;
    public static final int _QUEUE = 1;

    public static WorkingMemoryChangeQueueBehaviour
    convert(int val)
    {
        assert val >= 0 && val < 2;
        return values()[val];
    }

    public static WorkingMemoryChangeQueueBehaviour
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

    public static WorkingMemoryChangeQueueBehaviour
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(2);
        return WorkingMemoryChangeQueueBehaviour.convert(__v);
    }
}
