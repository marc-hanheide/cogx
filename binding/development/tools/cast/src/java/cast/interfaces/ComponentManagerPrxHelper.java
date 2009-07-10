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

public final class ComponentManagerPrxHelper extends Ice.ObjectPrxHelperBase implements ComponentManagerPrx
{
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
                _ComponentManagerDel __del = (_ComponentManagerDel)__delBase;
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

    public static ComponentManagerPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ComponentManagerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComponentManagerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::ComponentManager"))
                {
                    ComponentManagerPrxHelper __h = new ComponentManagerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ComponentManagerPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ComponentManagerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComponentManagerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::ComponentManager", __ctx))
                {
                    ComponentManagerPrxHelper __h = new ComponentManagerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ComponentManagerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ComponentManagerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::ComponentManager"))
                {
                    ComponentManagerPrxHelper __h = new ComponentManagerPrxHelper();
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

    public static ComponentManagerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ComponentManagerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::ComponentManager", __ctx))
                {
                    ComponentManagerPrxHelper __h = new ComponentManagerPrxHelper();
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

    public static ComponentManagerPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ComponentManagerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComponentManagerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ComponentManagerPrxHelper __h = new ComponentManagerPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ComponentManagerPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ComponentManagerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ComponentManagerPrxHelper __h = new ComponentManagerPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ComponentManagerDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ComponentManagerDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ComponentManagerPrx v)
    {
        __os.writeProxy(v);
    }

    public static ComponentManagerPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ComponentManagerPrxHelper result = new ComponentManagerPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
