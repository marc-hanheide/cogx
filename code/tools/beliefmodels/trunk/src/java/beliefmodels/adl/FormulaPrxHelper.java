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

public final class FormulaPrxHelper extends Ice.ObjectPrxHelperBase implements FormulaPrx
{
    public static FormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        FormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Formula"))
                {
                    FormulaPrxHelper __h = new FormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static FormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        FormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Formula", __ctx))
                {
                    FormulaPrxHelper __h = new FormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static FormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        FormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Formula"))
                {
                    FormulaPrxHelper __h = new FormulaPrxHelper();
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

    public static FormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        FormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Formula", __ctx))
                {
                    FormulaPrxHelper __h = new FormulaPrxHelper();
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

    public static FormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        FormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                FormulaPrxHelper __h = new FormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static FormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        FormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            FormulaPrxHelper __h = new FormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _FormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _FormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, FormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static FormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            FormulaPrxHelper result = new FormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
