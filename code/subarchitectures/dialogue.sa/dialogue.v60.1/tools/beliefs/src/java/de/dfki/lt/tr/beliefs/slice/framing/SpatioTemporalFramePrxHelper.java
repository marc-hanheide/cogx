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

public final class SpatioTemporalFramePrxHelper extends Ice.ObjectPrxHelperBase implements SpatioTemporalFramePrx
{
    public static SpatioTemporalFramePrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        SpatioTemporalFramePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SpatioTemporalFramePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::SpatioTemporalFrame"))
                {
                    SpatioTemporalFramePrxHelper __h = new SpatioTemporalFramePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SpatioTemporalFramePrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        SpatioTemporalFramePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SpatioTemporalFramePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::SpatioTemporalFrame", __ctx))
                {
                    SpatioTemporalFramePrxHelper __h = new SpatioTemporalFramePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SpatioTemporalFramePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SpatioTemporalFramePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::SpatioTemporalFrame"))
                {
                    SpatioTemporalFramePrxHelper __h = new SpatioTemporalFramePrxHelper();
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

    public static SpatioTemporalFramePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        SpatioTemporalFramePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::SpatioTemporalFrame", __ctx))
                {
                    SpatioTemporalFramePrxHelper __h = new SpatioTemporalFramePrxHelper();
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

    public static SpatioTemporalFramePrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        SpatioTemporalFramePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SpatioTemporalFramePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                SpatioTemporalFramePrxHelper __h = new SpatioTemporalFramePrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static SpatioTemporalFramePrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SpatioTemporalFramePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            SpatioTemporalFramePrxHelper __h = new SpatioTemporalFramePrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _SpatioTemporalFrameDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _SpatioTemporalFrameDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, SpatioTemporalFramePrx v)
    {
        __os.writeProxy(v);
    }

    public static SpatioTemporalFramePrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            SpatioTemporalFramePrxHelper result = new SpatioTemporalFramePrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
