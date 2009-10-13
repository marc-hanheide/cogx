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

public final class GoalFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements GoalFormulaPrx
{
    public static GoalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        GoalFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GoalFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::GoalFormula"))
                {
                    GoalFormulaPrxHelper __h = new GoalFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GoalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        GoalFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GoalFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::GoalFormula", __ctx))
                {
                    GoalFormulaPrxHelper __h = new GoalFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GoalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GoalFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::GoalFormula"))
                {
                    GoalFormulaPrxHelper __h = new GoalFormulaPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static GoalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        GoalFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::GoalFormula", __ctx))
                {
                    GoalFormulaPrxHelper __h = new GoalFormulaPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static GoalFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        GoalFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GoalFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                GoalFormulaPrxHelper __h = new GoalFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static GoalFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GoalFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            GoalFormulaPrxHelper __h = new GoalFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _GoalFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _GoalFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, GoalFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static GoalFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            GoalFormulaPrxHelper result = new GoalFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
