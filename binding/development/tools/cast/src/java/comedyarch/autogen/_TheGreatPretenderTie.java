// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package comedyarch.autogen;

public class _TheGreatPretenderTie extends _TheGreatPretenderDisp implements Ice.TieBase
{
    public
    _TheGreatPretenderTie()
    {
    }

    public
    _TheGreatPretenderTie(_TheGreatPretenderOperations delegate)
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
        _ice_delegate = (_TheGreatPretenderOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _TheGreatPretenderTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_TheGreatPretenderTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    public void
    getLies(Ice.Current __current)
    {
        _ice_delegate.getLies(__current);
    }

    private _TheGreatPretenderOperations _ice_delegate;
}
