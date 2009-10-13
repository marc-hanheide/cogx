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

public final class ContinualFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements ContinualFormulaPrx
{
    public static ContinualFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ContinualFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ContinualFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ContinualFormula"))
                {
                    ContinualFormulaPrxHelper __h = new ContinualFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ContinualFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ContinualFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ContinualFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ContinualFormula", __ctx))
                {
                    ContinualFormulaPrxHelper __h = new ContinualFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ContinualFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ContinualFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ContinualFormula"))
                {
                    ContinualFormulaPrxHelper __h = new ContinualFormulaPrxHelper();
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

    public static ContinualFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ContinualFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ContinualFormula", __ctx))
                {
                    ContinualFormulaPrxHelper __h = new ContinualFormulaPrxHelper();
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

    public static ContinualFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ContinualFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ContinualFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ContinualFormulaPrxHelper __h = new ContinualFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ContinualFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ContinualFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ContinualFormulaPrxHelper __h = new ContinualFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ContinualFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ContinualFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ContinualFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static ContinualFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ContinualFormulaPrxHelper result = new ContinualFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
