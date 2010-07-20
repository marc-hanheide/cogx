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

public final class SharedEpistemicStatusPrxHelper extends Ice.ObjectPrxHelperBase implements SharedEpistemicStatusPrx
{
    public static SharedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        SharedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SharedEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::SharedEpistemicStatus"))
                {
                    SharedEpistemicStatusPrxHelper __h = new SharedEpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SharedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        SharedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SharedEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::SharedEpistemicStatus", __ctx))
                {
                    SharedEpistemicStatusPrxHelper __h = new SharedEpistemicStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SharedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SharedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::SharedEpistemicStatus"))
                {
                    SharedEpistemicStatusPrxHelper __h = new SharedEpistemicStatusPrxHelper();
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

    public static SharedEpistemicStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        SharedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epstatus::SharedEpistemicStatus", __ctx))
                {
                    SharedEpistemicStatusPrxHelper __h = new SharedEpistemicStatusPrxHelper();
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

    public static SharedEpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        SharedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SharedEpistemicStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                SharedEpistemicStatusPrxHelper __h = new SharedEpistemicStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static SharedEpistemicStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SharedEpistemicStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            SharedEpistemicStatusPrxHelper __h = new SharedEpistemicStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _SharedEpistemicStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _SharedEpistemicStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, SharedEpistemicStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static SharedEpistemicStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            SharedEpistemicStatusPrxHelper result = new SharedEpistemicStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
