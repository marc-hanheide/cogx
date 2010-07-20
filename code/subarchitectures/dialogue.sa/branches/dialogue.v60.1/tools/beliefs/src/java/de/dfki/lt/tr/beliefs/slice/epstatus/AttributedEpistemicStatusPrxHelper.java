// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.epstatus;

public final class AttributedEpistemicStatusPrxHelper extends Ice.ObjectPrxHelperBase implements AttributedEpistemicStatusPrx
{
    public static AttributedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        AttributedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AttributedEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::AttributedEpistemicStatus"))
                {
                    AttributedEpistemicStatusPrxHelper __h = new AttributedEpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AttributedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        AttributedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AttributedEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::AttributedEpistemicStatus", __ctx))
                {
                    AttributedEpistemicStatusPrxHelper __h = new AttributedEpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AttributedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AttributedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::AttributedEpistemicStatus"))
                {
                    AttributedEpistemicStatusPrxHelper __h = new AttributedEpistemicStatusPrxHelper();
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

    public static AttributedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        AttributedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::AttributedEpistemicStatus", __ctx))
                {
                    AttributedEpistemicStatusPrxHelper __h = new AttributedEpistemicStatusPrxHelper();
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

    public static AttributedEpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        AttributedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AttributedEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                AttributedEpistemicStatusPrxHelper __h = new AttributedEpistemicStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static AttributedEpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AttributedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            AttributedEpistemicStatusPrxHelper __h = new AttributedEpistemicStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _AttributedEpistemicStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _AttributedEpistemicStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, AttributedEpistemicStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static AttributedEpistemicStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            AttributedEpistemicStatusPrxHelper result = new AttributedEpistemicStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
