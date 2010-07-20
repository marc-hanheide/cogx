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

public final class CondIndependentDistribsPrxHelper extends Ice.ObjectPrxHelperBase implements CondIndependentDistribsPrx
{
    public static CondIndependentDistribsPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        CondIndependentDistribsPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (CondIndependentDistribsPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::CondIndependentDistribs"))
                {
                    CondIndependentDistribsPrxHelper __h = new CondIndependentDistribsPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static CondIndependentDistribsPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        CondIndependentDistribsPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (CondIndependentDistribsPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::CondIndependentDistribs", __ctx))
                {
                    CondIndependentDistribsPrxHelper __h = new CondIndependentDistribsPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static CondIndependentDistribsPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        CondIndependentDistribsPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::CondIndependentDistribs"))
                {
                    CondIndependentDistribsPrxHelper __h = new CondIndependentDistribsPrxHelper();
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

    public static CondIndependentDistribsPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        CondIndependentDistribsPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::CondIndependentDistribs", __ctx))
                {
                    CondIndependentDistribsPrxHelper __h = new CondIndependentDistribsPrxHelper();
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

    public static CondIndependentDistribsPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        CondIndependentDistribsPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (CondIndependentDistribsPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                CondIndependentDistribsPrxHelper __h = new CondIndependentDistribsPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static CondIndependentDistribsPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        CondIndependentDistribsPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            CondIndependentDistribsPrxHelper __h = new CondIndependentDistribsPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _CondIndependentDistribsDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _CondIndependentDistribsDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, CondIndependentDistribsPrx v)
    {
        __os.writeProxy(v);
    }

    public static CondIndependentDistribsPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            CondIndependentDistribsPrxHelper result = new CondIndependentDistribsPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
