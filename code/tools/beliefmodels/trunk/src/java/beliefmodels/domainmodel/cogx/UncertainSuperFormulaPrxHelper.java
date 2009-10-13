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

public final class UncertainSuperFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements UncertainSuperFormulaPrx
{
    public static UncertainSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        UncertainSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UncertainSuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::UncertainSuperFormula"))
                {
                    UncertainSuperFormulaPrxHelper __h = new UncertainSuperFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UncertainSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        UncertainSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UncertainSuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::UncertainSuperFormula", __ctx))
                {
                    UncertainSuperFormulaPrxHelper __h = new UncertainSuperFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UncertainSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UncertainSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::UncertainSuperFormula"))
                {
                    UncertainSuperFormulaPrxHelper __h = new UncertainSuperFormulaPrxHelper();
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

    public static UncertainSuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        UncertainSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::UncertainSuperFormula", __ctx))
                {
                    UncertainSuperFormulaPrxHelper __h = new UncertainSuperFormulaPrxHelper();
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

    public static UncertainSuperFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        UncertainSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UncertainSuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                UncertainSuperFormulaPrxHelper __h = new UncertainSuperFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static UncertainSuperFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UncertainSuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            UncertainSuperFormulaPrxHelper __h = new UncertainSuperFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _UncertainSuperFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _UncertainSuperFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, UncertainSuperFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static UncertainSuperFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            UncertainSuperFormulaPrxHelper result = new UncertainSuperFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
