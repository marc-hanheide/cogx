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

public final class NegatedFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements NegatedFormulaPrx
{
    public static NegatedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        NegatedFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (NegatedFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::NegatedFormula"))
                {
                    NegatedFormulaPrxHelper __h = new NegatedFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static NegatedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        NegatedFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (NegatedFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::NegatedFormula", __ctx))
                {
                    NegatedFormulaPrxHelper __h = new NegatedFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static NegatedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        NegatedFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::NegatedFormula"))
                {
                    NegatedFormulaPrxHelper __h = new NegatedFormulaPrxHelper();
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

    public static NegatedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        NegatedFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::NegatedFormula", __ctx))
                {
                    NegatedFormulaPrxHelper __h = new NegatedFormulaPrxHelper();
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

    public static NegatedFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        NegatedFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (NegatedFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                NegatedFormulaPrxHelper __h = new NegatedFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static NegatedFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        NegatedFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            NegatedFormulaPrxHelper __h = new NegatedFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _NegatedFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _NegatedFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, NegatedFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static NegatedFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            NegatedFormulaPrxHelper result = new NegatedFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
