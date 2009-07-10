// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.interfaces;

public class _ComponentManagerTie extends _ComponentManagerDisp implements Ice.TieBase
{
    public
    _ComponentManagerTie()
    {
    }

    public
    _ComponentManagerTie(_ComponentManagerOperations delegate)
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
        _ice_delegate = (_ComponentManagerOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _ComponentManagerTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_ComponentManagerTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    public cast.cdl.CASTTime
    getCASTTime(Ice.Current __current)
    {
        return _ice_delegate.getCASTTime(__current);
    }

    private _ComponentManagerOperations _ice_delegate;
}
