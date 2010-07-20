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

public final class BooleanFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements BooleanFormulaPrx
{
    public static BooleanFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        BooleanFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BooleanFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::BooleanFormula"))
                {
                    BooleanFormulaPrxHelper __h = new BooleanFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BooleanFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        BooleanFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BooleanFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::BooleanFormula", __ctx))
                {
                    BooleanFormulaPrxHelper __h = new BooleanFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BooleanFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BooleanFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::BooleanFormula"))
                {
                    BooleanFormulaPrxHelper __h = new BooleanFormulaPrxHelper();
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

    public static BooleanFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        BooleanFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::BooleanFormula", __ctx))
                {
                    BooleanFormulaPrxHelper __h = new BooleanFormulaPrxHelper();
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

    public static BooleanFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        BooleanFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BooleanFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                BooleanFormulaPrxHelper __h = new BooleanFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static BooleanFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BooleanFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            BooleanFormulaPrxHelper __h = new BooleanFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _BooleanFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _BooleanFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, BooleanFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static BooleanFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            BooleanFormulaPrxHelper result = new BooleanFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
