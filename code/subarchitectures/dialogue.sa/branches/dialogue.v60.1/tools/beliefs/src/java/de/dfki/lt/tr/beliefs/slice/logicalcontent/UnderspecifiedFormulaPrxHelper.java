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

public final class UnderspecifiedFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements UnderspecifiedFormulaPrx
{
    public static UnderspecifiedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        UnderspecifiedFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnderspecifiedFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnderspecifiedFormula"))
                {
                    UnderspecifiedFormulaPrxHelper __h = new UnderspecifiedFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UnderspecifiedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        UnderspecifiedFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnderspecifiedFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnderspecifiedFormula", __ctx))
                {
                    UnderspecifiedFormulaPrxHelper __h = new UnderspecifiedFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UnderspecifiedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UnderspecifiedFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnderspecifiedFormula"))
                {
                    UnderspecifiedFormulaPrxHelper __h = new UnderspecifiedFormulaPrxHelper();
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

    public static UnderspecifiedFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        UnderspecifiedFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnderspecifiedFormula", __ctx))
                {
                    UnderspecifiedFormulaPrxHelper __h = new UnderspecifiedFormulaPrxHelper();
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

    public static UnderspecifiedFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        UnderspecifiedFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnderspecifiedFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                UnderspecifiedFormulaPrxHelper __h = new UnderspecifiedFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static UnderspecifiedFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UnderspecifiedFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            UnderspecifiedFormulaPrxHelper __h = new UnderspecifiedFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _UnderspecifiedFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _UnderspecifiedFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, UnderspecifiedFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static UnderspecifiedFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            UnderspecifiedFormulaPrxHelper result = new UnderspecifiedFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
