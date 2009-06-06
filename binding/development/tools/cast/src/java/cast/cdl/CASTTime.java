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

public final class CASTTime implements java.lang.Cloneable, java.io.Serializable
{
    public long s;

    public long us;

    public CASTTime()
    {
    }

    public CASTTime(long s, long us)
    {
        this.s = s;
        this.us = us;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        CASTTime _r = null;
        try
        {
            _r = (CASTTime)rhs;
        }
        catch(ClassCastException ex)
        {
        }

        if(_r != null)
        {
            if(s != _r.s)
            {
                return false;
            }
            if(us != _r.us)
            {
                return false;
            }

            return true;
        }

        return false;
    }

    public int
    hashCode()
    {
        int __h = 0;
        __h = 5 * __h + (int)s;
        __h = 5 * __h + (int)us;
        return __h;
    }

    public java.lang.Object
    clone()
    {
        java.lang.Object o = null;
        try
        {
            o = super.clone();
        }
        catch(CloneNotSupportedException ex)
        {
            assert false; // impossible
        }
        return o;
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeLong(s);
        __os.writeLong(us);
    }

    public void
    __read(IceInternal.BasicStream __is)
    {
        s = __is.readLong();
        us = __is.readLong();
    }
}
