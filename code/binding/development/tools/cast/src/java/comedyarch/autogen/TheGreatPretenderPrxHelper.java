// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package comedyarch.autogen;

public final class TheGreatPretenderPrxHelper extends Ice.ObjectPrxHelperBase implements TheGreatPretenderPrx
{
    public void
    getLies()
    {
        getLies(null, false);
    }

    public void
    getLies(java.util.Map<String, String> __ctx)
    {
        getLies(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    getLies(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _TheGreatPretenderDel __del = (_TheGreatPretenderDel)__delBase;
                __del.getLies(__ctx);
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

    public static TheGreatPretenderPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TheGreatPretenderPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TheGreatPretenderPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::TheGreatPretender"))
                {
                    TheGreatPretenderPrxHelper __h = new TheGreatPretenderPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TheGreatPretenderPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TheGreatPretenderPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TheGreatPretenderPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::TheGreatPretender", __ctx))
                {
                    TheGreatPretenderPrxHelper __h = new TheGreatPretenderPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TheGreatPretenderPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TheGreatPretenderPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::TheGreatPretender"))
                {
                    TheGreatPretenderPrxHelper __h = new TheGreatPretenderPrxHelper();
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

    public static TheGreatPretenderPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TheGreatPretenderPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::TheGreatPretender", __ctx))
                {
                    TheGreatPretenderPrxHelper __h = new TheGreatPretenderPrxHelper();
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

    public static TheGreatPretenderPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TheGreatPretenderPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TheGreatPretenderPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TheGreatPretenderPrxHelper __h = new TheGreatPretenderPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TheGreatPretenderPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TheGreatPretenderPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TheGreatPretenderPrxHelper __h = new TheGreatPretenderPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TheGreatPretenderDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TheGreatPretenderDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TheGreatPretenderPrx v)
    {
        __os.writeProxy(v);
    }

    public static TheGreatPretenderPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TheGreatPretenderPrxHelper result = new TheGreatPretenderPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
