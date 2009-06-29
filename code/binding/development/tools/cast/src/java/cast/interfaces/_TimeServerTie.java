// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0

package cast.interfaces;

public class _TimeServerTie extends _TimeServerDisp implements Ice.TieBase
{
    public
    _TimeServerTie()
    {
    }

    public
    _TimeServerTie(_TimeServerOperations delegate)
    {
        _ice_delegate = delegate;
    }

    public java.lang.Object
    ice_delegate()
    {
        return _ice_delegate;
    }

    public void
    ice_delegate(java.lang.Object delegate)
    {
        _ice_delegate = (_TimeServerOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _TimeServerTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_TimeServerTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    public cast.cdl.CASTTime
    fromTimeOfDay(long secs, long usecs, Ice.Current __current)
    {
        return _ice_delegate.fromTimeOfDay(secs, usecs, __current);
    }

    public cast.cdl.CASTTime
    fromTimeOfDayDouble(double todsecs, Ice.Current __current)
    {
        return _ice_delegate.fromTimeOfDayDouble(todsecs, __current);
    }

    public cast.cdl.CASTTime
    getCASTTime(Ice.Current __current)
    {
        return _ice_delegate.getCASTTime(__current);
    }

    public void
    reset(Ice.Current __current)
    {
        _ice_delegate.reset(__current);
    }

    private _TimeServerOperations _ice_delegate;
}
