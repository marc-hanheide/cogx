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

public final class EpistemicStatusPrxHelper extends Ice.ObjectPrxHelperBase implements EpistemicStatusPrx
{
    public static EpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        EpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::EpistemicStatus"))
                {
                    EpistemicStatusPrxHelper __h = new EpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        EpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::EpistemicStatus", __ctx))
                {
                    EpistemicStatusPrxHelper __h = new EpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::EpistemicStatus"))
                {
                    EpistemicStatusPrxHelper __h = new EpistemicStatusPrxHelper();
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

    public static EpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        EpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::EpistemicStatus", __ctx))
                {
                    EpistemicStatusPrxHelper __h = new EpistemicStatusPrxHelper();
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

    public static EpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        EpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                EpistemicStatusPrxHelper __h = new EpistemicStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static EpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            EpistemicStatusPrxHelper __h = new EpistemicStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _EpistemicStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _EpistemicStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, EpistemicStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static EpistemicStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            EpistemicStatusPrxHelper result = new EpistemicStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
