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

public enum TaskOutcome implements java.io.Serializable
{
    ProcessingIncomplete,
    ProcessingComplete,
    ProcessingCompleteSuccess,
    ProcessingCompleteFailure;

    public static final int _ProcessingIncomplete = 0;
    public static final int _ProcessingComplete = 1;
    public static final int _ProcessingCompleteSuccess = 2;
    public static final int _ProcessingCompleteFailure = 3;

    public static TaskOutcome
    convert(int val)
    {
        assert val >= 0 && val < 4;
        return values()[val];
    }

    public static TaskOutcome
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

    public static TaskOutcome
    __read(IceInternal.BasicStream __is)
    {
        int __v = __is.readByte(4);
        return TaskOutcome.convert(__v);
    }
}
