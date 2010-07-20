// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.history;

public final class AbstractBeliefHistoryPrxHelper extends Ice.ObjectPrxHelperBase implements AbstractBeliefHistoryPrx
{
    public static AbstractBeliefHistoryPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        AbstractBeliefHistoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AbstractBeliefHistoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::history::AbstractBeliefHistory"))
                {
                    AbstractBeliefHistoryPrxHelper __h = new AbstractBeliefHistoryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AbstractBeliefHistoryPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        AbstractBeliefHistoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AbstractBeliefHistoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::history::AbstractBeliefHistory", __ctx))
                {
                    AbstractBeliefHistoryPrxHelper __h = new AbstractBeliefHistoryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AbstractBeliefHistoryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AbstractBeliefHistoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::history::AbstractBeliefHistory"))
                {
                    AbstractBeliefHistoryPrxHelper __h = new AbstractBeliefHistoryPrxHelper();
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

    public static AbstractBeliefHistoryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        AbstractBeliefHistoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::history::AbstractBeliefHistory", __ctx))
                {
                    AbstractBeliefHistoryPrxHelper __h = new AbstractBeliefHistoryPrxHelper();
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

    public static AbstractBeliefHistoryPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        AbstractBeliefHistoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AbstractBeliefHistoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                AbstractBeliefHistoryPrxHelper __h = new AbstractBeliefHistoryPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static AbstractBeliefHistoryPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AbstractBeliefHistoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            AbstractBeliefHistoryPrxHelper __h = new AbstractBeliefHistoryPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _AbstractBeliefHistoryDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _AbstractBeliefHistoryDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, AbstractBeliefHistoryPrx v)
    {
        __os.writeProxy(v);
    }

    public static AbstractBeliefHistoryPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            AbstractBeliefHistoryPrxHelper result = new AbstractBeliefHistoryPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
