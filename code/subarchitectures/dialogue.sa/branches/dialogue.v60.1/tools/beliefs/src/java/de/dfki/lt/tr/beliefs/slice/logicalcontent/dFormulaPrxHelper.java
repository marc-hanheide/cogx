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

public final class dFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements dFormulaPrx
{
    public static dFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        dFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (dFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula"))
                {
                    dFormulaPrxHelper __h = new dFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static dFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        dFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (dFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula", __ctx))
                {
                    dFormulaPrxHelper __h = new dFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static dFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        dFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula"))
                {
                    dFormulaPrxHelper __h = new dFormulaPrxHelper();
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

    public static dFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        dFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula", __ctx))
                {
                    dFormulaPrxHelper __h = new dFormulaPrxHelper();
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

    public static dFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        dFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (dFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                dFormulaPrxHelper __h = new dFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static dFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        dFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            dFormulaPrxHelper __h = new dFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _dFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _dFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, dFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static dFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            dFormulaPrxHelper result = new dFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
