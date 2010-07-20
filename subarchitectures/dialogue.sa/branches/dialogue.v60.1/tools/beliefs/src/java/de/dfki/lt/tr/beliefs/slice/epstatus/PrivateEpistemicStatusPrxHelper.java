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

public final class PrivateEpistemicStatusPrxHelper extends Ice.ObjectPrxHelperBase implements PrivateEpistemicStatusPrx
{
    public static PrivateEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        PrivateEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PrivateEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::PrivateEpistemicStatus"))
                {
                    PrivateEpistemicStatusPrxHelper __h = new PrivateEpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static PrivateEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        PrivateEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PrivateEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::PrivateEpistemicStatus", __ctx))
                {
                    PrivateEpistemicStatusPrxHelper __h = new PrivateEpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static PrivateEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        PrivateEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::PrivateEpistemicStatus"))
                {
                    PrivateEpistemicStatusPrxHelper __h = new PrivateEpistemicStatusPrxHelper();
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

    public static PrivateEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        PrivateEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::PrivateEpistemicStatus", __ctx))
                {
                    PrivateEpistemicStatusPrxHelper __h = new PrivateEpistemicStatusPrxHelper();
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

    public static PrivateEpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        PrivateEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PrivateEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                PrivateEpistemicStatusPrxHelper __h = new PrivateEpistemicStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static PrivateEpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        PrivateEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            PrivateEpistemicStatusPrxHelper __h = new PrivateEpistemicStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _PrivateEpistemicStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _PrivateEpistemicStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, PrivateEpistemicStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static PrivateEpistemicStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            PrivateEpistemicStatusPrxHelper result = new PrivateEpistemicStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
