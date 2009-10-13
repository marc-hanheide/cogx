// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.domainmodel.cogx;

public final class LogicalSuperFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements LogicalSuperFormulaPrx
{
    public static LogicalSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        LogicalSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LogicalSuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LogicalSuperFormula"))
                {
                    LogicalSuperFormulaPrxHelper __h = new LogicalSuperFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LogicalSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        LogicalSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LogicalSuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LogicalSuperFormula", __ctx))
                {
                    LogicalSuperFormulaPrxHelper __h = new LogicalSuperFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LogicalSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LogicalSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LogicalSuperFormula"))
                {
                    LogicalSuperFormulaPrxHelper __h = new LogicalSuperFormulaPrxHelper();
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

    public static LogicalSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        LogicalSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LogicalSuperFormula", __ctx))
                {
                    LogicalSuperFormulaPrxHelper __h = new LogicalSuperFormulaPrxHelper();
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

    public static LogicalSuperFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        LogicalSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LogicalSuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                LogicalSuperFormulaPrxHelper __h = new LogicalSuperFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static LogicalSuperFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LogicalSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            LogicalSuperFormulaPrxHelper __h = new LogicalSuperFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _LogicalSuperFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _LogicalSuperFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, LogicalSuperFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static LogicalSuperFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            LogicalSuperFormulaPrxHelper result = new LogicalSuperFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
