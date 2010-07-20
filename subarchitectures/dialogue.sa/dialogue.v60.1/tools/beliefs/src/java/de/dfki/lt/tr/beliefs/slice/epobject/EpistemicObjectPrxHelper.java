// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.epobject;

public final class EpistemicObjectPrxHelper extends Ice.ObjectPrxHelperBase implements EpistemicObjectPrx
{
    public static EpistemicObjectPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        EpistemicObjectPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicObjectPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epobject::EpistemicObject"))
                {
                    EpistemicObjectPrxHelper __h = new EpistemicObjectPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EpistemicObjectPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        EpistemicObjectPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicObjectPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::epobject::EpistemicObject", __ctx))
                {
                    EpistemicObjectPrxHelper __h = new EpistemicObjectPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EpistemicObjectPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EpistemicObjectPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epobject::EpistemicObject"))
                {
                    EpistemicObjectPrxHelper __h = new EpistemicObjectPrxHelper();
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

    public static EpistemicObjectPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        EpistemicObjectPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::epobject::EpistemicObject", __ctx))
                {
                    EpistemicObjectPrxHelper __h = new EpistemicObjectPrxHelper();
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

    public static EpistemicObjectPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        EpistemicObjectPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicObjectPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                EpistemicObjectPrxHelper __h = new EpistemicObjectPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static EpistemicObjectPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EpistemicObjectPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            EpistemicObjectPrxHelper __h = new EpistemicObjectPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _EpistemicObjectDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _EpistemicObjectDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, EpistemicObjectPrx v)
    {
        __os.writeProxy(v);
    }

    public static EpistemicObjectPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            EpistemicObjectPrxHelper result = new EpistemicObjectPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
