// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.distribs;

public final class NormalValuesPrxHelper extends Ice.ObjectPrxHelperBase implements NormalValuesPrx
{
    public static NormalValuesPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        NormalValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (NormalValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::NormalValues"))
                {
                    NormalValuesPrxHelper __h = new NormalValuesPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static NormalValuesPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        NormalValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (NormalValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::NormalValues", __ctx))
                {
                    NormalValuesPrxHelper __h = new NormalValuesPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static NormalValuesPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        NormalValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::NormalValues"))
                {
                    NormalValuesPrxHelper __h = new NormalValuesPrxHelper();
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

    public static NormalValuesPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        NormalValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::NormalValues", __ctx))
                {
                    NormalValuesPrxHelper __h = new NormalValuesPrxHelper();
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

    public static NormalValuesPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        NormalValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (NormalValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                NormalValuesPrxHelper __h = new NormalValuesPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static NormalValuesPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        NormalValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            NormalValuesPrxHelper __h = new NormalValuesPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _NormalValuesDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _NormalValuesDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, NormalValuesPrx v)
    {
        __os.writeProxy(v);
    }

    public static NormalValuesPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            NormalValuesPrxHelper result = new NormalValuesPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
