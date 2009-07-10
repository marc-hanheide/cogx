// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.interfaces;

public final class TimeServerPrxHelper extends Ice.ObjectPrxHelperBase implements TimeServerPrx
{
    public cast.cdl.CASTTime
    fromTimeOfDay(long secs, long usecs)
    {
        return fromTimeOfDay(secs, usecs, null, false);
    }

    public cast.cdl.CASTTime
    fromTimeOfDay(long secs, long usecs, java.util.Map<String, String> __ctx)
    {
        return fromTimeOfDay(secs, usecs, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private cast.cdl.CASTTime
    fromTimeOfDay(long secs, long usecs, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("fromTimeOfDay");
                __delBase = __getDelegate(false);
                _TimeServerDel __del = (_TimeServerDel)__delBase;
                return __del.fromTimeOfDay(secs, usecs, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public cast.cdl.CASTTime
    fromTimeOfDayDouble(double todsecs)
    {
        return fromTimeOfDayDouble(todsecs, null, false);
    }

    public cast.cdl.CASTTime
    fromTimeOfDayDouble(double todsecs, java.util.Map<String, String> __ctx)
    {
        return fromTimeOfDayDouble(todsecs, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private cast.cdl.CASTTime
    fromTimeOfDayDouble(double todsecs, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("fromTimeOfDayDouble");
                __delBase = __getDelegate(false);
                _TimeServerDel __del = (_TimeServerDel)__delBase;
                return __del.fromTimeOfDayDouble(todsecs, __ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public cast.cdl.CASTTime
    getCASTTime()
    {
        return getCASTTime(null, false);
    }

    public cast.cdl.CASTTime
    getCASTTime(java.util.Map<String, String> __ctx)
    {
        return getCASTTime(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private cast.cdl.CASTTime
    getCASTTime(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __checkTwowayOnly("getCASTTime");
                __delBase = __getDelegate(false);
                _TimeServerDel __del = (_TimeServerDel)__delBase;
                return __del.getCASTTime(__ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    reset()
    {
        reset(null, false);
    }

    public void
    reset(java.util.Map<String, String> __ctx)
    {
        reset(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    reset(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        int __cnt = 0;
        while(true)
        {
            Ice._ObjectDel __delBase = null;
            try
            {
                __delBase = __getDelegate(false);
                _TimeServerDel __del = (_TimeServerDel)__delBase;
                __del.reset(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __handleExceptionWrapper(__delBase, __ex, null);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public static TimeServerPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TimeServerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TimeServerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::TimeServer"))
                {
                    TimeServerPrxHelper __h = new TimeServerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TimeServerPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TimeServerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TimeServerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::TimeServer", __ctx))
                {
                    TimeServerPrxHelper __h = new TimeServerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TimeServerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TimeServerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::TimeServer"))
                {
                    TimeServerPrxHelper __h = new TimeServerPrxHelper();
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

    public static TimeServerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TimeServerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::TimeServer", __ctx))
                {
                    TimeServerPrxHelper __h = new TimeServerPrxHelper();
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

    public static TimeServerPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TimeServerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TimeServerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TimeServerPrxHelper __h = new TimeServerPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TimeServerPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TimeServerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TimeServerPrxHelper __h = new TimeServerPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TimeServerDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TimeServerDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TimeServerPrx v)
    {
        __os.writeProxy(v);
    }

    public static TimeServerPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TimeServerPrxHelper result = new TimeServerPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
