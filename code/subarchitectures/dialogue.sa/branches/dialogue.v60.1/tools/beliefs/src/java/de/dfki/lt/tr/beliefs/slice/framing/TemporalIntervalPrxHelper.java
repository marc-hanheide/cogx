// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.framing;

public final class TemporalIntervalPrxHelper extends Ice.ObjectPrxHelperBase implements TemporalIntervalPrx
{
    public static TemporalIntervalPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TemporalIntervalPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TemporalIntervalPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::TemporalInterval"))
                {
                    TemporalIntervalPrxHelper __h = new TemporalIntervalPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TemporalIntervalPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TemporalIntervalPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TemporalIntervalPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::TemporalInterval", __ctx))
                {
                    TemporalIntervalPrxHelper __h = new TemporalIntervalPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TemporalIntervalPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TemporalIntervalPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::TemporalInterval"))
                {
                    TemporalIntervalPrxHelper __h = new TemporalIntervalPrxHelper();
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

    public static TemporalIntervalPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TemporalIntervalPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::TemporalInterval", __ctx))
                {
                    TemporalIntervalPrxHelper __h = new TemporalIntervalPrxHelper();
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

    public static TemporalIntervalPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TemporalIntervalPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TemporalIntervalPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TemporalIntervalPrxHelper __h = new TemporalIntervalPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TemporalIntervalPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TemporalIntervalPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TemporalIntervalPrxHelper __h = new TemporalIntervalPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TemporalIntervalDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TemporalIntervalDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TemporalIntervalPrx v)
    {
        __os.writeProxy(v);
    }

    public static TemporalIntervalPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TemporalIntervalPrxHelper result = new TemporalIntervalPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
