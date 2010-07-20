// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.logicalcontent;

public final class ComplexFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements ComplexFormulaPrx
{
    public static ComplexFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ComplexFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComplexFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ComplexFormula"))
                {
                    ComplexFormulaPrxHelper __h = new ComplexFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ComplexFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ComplexFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComplexFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ComplexFormula", __ctx))
                {
                    ComplexFormulaPrxHelper __h = new ComplexFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ComplexFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ComplexFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ComplexFormula"))
                {
                    ComplexFormulaPrxHelper __h = new ComplexFormulaPrxHelper();
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

    public static ComplexFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ComplexFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ComplexFormula", __ctx))
                {
                    ComplexFormulaPrxHelper __h = new ComplexFormulaPrxHelper();
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

    public static ComplexFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ComplexFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComplexFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ComplexFormulaPrxHelper __h = new ComplexFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ComplexFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ComplexFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ComplexFormulaPrxHelper __h = new ComplexFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ComplexFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ComplexFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ComplexFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static ComplexFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ComplexFormulaPrxHelper result = new ComplexFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
