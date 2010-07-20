// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.events;

public final class EventPrxHelper extends Ice.ObjectPrxHelperBase implements EventPrx
{
    public static EventPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        EventPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EventPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::events::Event"))
                {
                    EventPrxHelper __h = new EventPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EventPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        EventPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EventPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::events::Event", __ctx))
                {
                    EventPrxHelper __h = new EventPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EventPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EventPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::events::Event"))
                {
                    EventPrxHelper __h = new EventPrxHelper();
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

    public static EventPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        EventPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::events::Event", __ctx))
                {
                    EventPrxHelper __h = new EventPrxHelper();
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

    public static EventPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        EventPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EventPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                EventPrxHelper __h = new EventPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static EventPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EventPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            EventPrxHelper __h = new EventPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _EventDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _EventDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, EventPrx v)
    {
        __os.writeProxy(v);
    }

    public static EventPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            EventPrxHelper result = new EventPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
