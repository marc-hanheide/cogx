// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.examples.autogen;

public class _WordServerTie extends _WordServerDisp implements Ice.TieBase
{
    public
    _WordServerTie()
    {
    }

    public
    _WordServerTie(_WordServerOperations delegate)
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
        _ice_delegate = (_WordServerOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _WordServerTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_WordServerTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    public String
    getNewWord(Ice.Current __current)
    {
        return _ice_delegate.getNewWord(__current);
    }

    private _WordServerOperations _ice_delegate;
}
