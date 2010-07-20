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

public final class DistributionValuesPrxHelper extends Ice.ObjectPrxHelperBase implements DistributionValuesPrx
{
    public static DistributionValuesPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        DistributionValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DistributionValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::DistributionValues"))
                {
                    DistributionValuesPrxHelper __h = new DistributionValuesPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static DistributionValuesPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        DistributionValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DistributionValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::DistributionValues", __ctx))
                {
                    DistributionValuesPrxHelper __h = new DistributionValuesPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static DistributionValuesPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        DistributionValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::DistributionValues"))
                {
                    DistributionValuesPrxHelper __h = new DistributionValuesPrxHelper();
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

    public static DistributionValuesPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        DistributionValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::DistributionValues", __ctx))
                {
                    DistributionValuesPrxHelper __h = new DistributionValuesPrxHelper();
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

    public static DistributionValuesPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        DistributionValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DistributionValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                DistributionValuesPrxHelper __h = new DistributionValuesPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static DistributionValuesPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        DistributionValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            DistributionValuesPrxHelper __h = new DistributionValuesPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _DistributionValuesDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _DistributionValuesDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, DistributionValuesPrx v)
    {
        __os.writeProxy(v);
    }

    public static DistributionValuesPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            DistributionValuesPrxHelper result = new DistributionValuesPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
