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

public final class SuperFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements SuperFormulaPrx
{
    public static SuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        SuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::SuperFormula"))
                {
                    SuperFormulaPrxHelper __h = new SuperFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        SuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::SuperFormula", __ctx))
                {
                    SuperFormulaPrxHelper __h = new SuperFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::SuperFormula"))
                {
                    SuperFormulaPrxHelper __h = new SuperFormulaPrxHelper();
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

    public static SuperFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        SuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::SuperFormula", __ctx))
                {
                    SuperFormulaPrxHelper __h = new SuperFormulaPrxHelper();
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

    public static SuperFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        SuperFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SuperFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                SuperFormulaPrxHelper __h = new SuperFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static SuperFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SuperFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            SuperFormulaPrxHelper __h = new SuperFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _SuperFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _SuperFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, SuperFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static SuperFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            SuperFormulaPrxHelper result = new SuperFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
