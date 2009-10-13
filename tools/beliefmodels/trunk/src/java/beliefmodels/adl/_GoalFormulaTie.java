// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.adl;

public class _GoalFormulaTie extends _GoalFormulaDisp implements Ice.TieBase
{
    public
    _GoalFormulaTie()
    {
    }

    public
    _GoalFormulaTie(_GoalFormulaOperations delegate)
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
        _ice_delegate = (_GoalFormulaOperations)delegate;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        if(!(rhs instanceof _GoalFormulaTie))
        {
            return false;
        }

        return _ice_delegate.equals(((_GoalFormulaTie)rhs)._ice_delegate);
    }

    public int
    hashCode()
    {
        return _ice_delegate.hashCode();
    }

    private _GoalFormulaOperations _ice_delegate;
}
