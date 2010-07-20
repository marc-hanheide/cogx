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

public final class AbstractFramePrxHelper extends Ice.ObjectPrxHelperBase implements AbstractFramePrx
{
    public static AbstractFramePrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        AbstractFramePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AbstractFramePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::AbstractFrame"))
                {
                    AbstractFramePrxHelper __h = new AbstractFramePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AbstractFramePrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        AbstractFramePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AbstractFramePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::AbstractFrame", __ctx))
                {
                    AbstractFramePrxHelper __h = new AbstractFramePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AbstractFramePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AbstractFramePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::AbstractFrame"))
                {
                    AbstractFramePrxHelper __h = new AbstractFramePrxHelper();
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

    public static AbstractFramePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        AbstractFramePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::framing::AbstractFrame", __ctx))
                {
                    AbstractFramePrxHelper __h = new AbstractFramePrxHelper();
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

    public static AbstractFramePrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        AbstractFramePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AbstractFramePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                AbstractFramePrxHelper __h = new AbstractFramePrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static AbstractFramePrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AbstractFramePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            AbstractFramePrxHelper __h = new AbstractFramePrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _AbstractFrameDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _AbstractFrameDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, AbstractFramePrx v)
    {
        __os.writeProxy(v);
    }

    public static AbstractFramePrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            AbstractFramePrxHelper result = new AbstractFramePrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
