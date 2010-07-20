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

public final class FloatFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements FloatFormulaPrx
{
    public static FloatFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        FloatFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FloatFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::FloatFormula"))
                {
                    FloatFormulaPrxHelper __h = new FloatFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static FloatFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        FloatFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FloatFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::FloatFormula", __ctx))
                {
                    FloatFormulaPrxHelper __h = new FloatFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static FloatFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        FloatFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::FloatFormula"))
                {
                    FloatFormulaPrxHelper __h = new FloatFormulaPrxHelper();
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

    public static FloatFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        FloatFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::FloatFormula", __ctx))
                {
                    FloatFormulaPrxHelper __h = new FloatFormulaPrxHelper();
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

    public static FloatFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        FloatFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FloatFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                FloatFormulaPrxHelper __h = new FloatFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static FloatFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        FloatFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            FloatFormulaPrxHelper __h = new FloatFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _FloatFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _FloatFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, FloatFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static FloatFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            FloatFormulaPrxHelper result = new FloatFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
