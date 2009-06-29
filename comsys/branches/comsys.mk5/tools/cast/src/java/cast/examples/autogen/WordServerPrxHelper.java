// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.examples.autogen;

public final class WordServerPrxHelper extends Ice.ObjectPrxHelperBase implements WordServerPrx
{
    public String
    getNewWord()
    {
        return getNewWord(null, false);
    }

    public String
    getNewWord(java.util.Map<String, String> __ctx)
    {
        return getNewWord(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private String
    getNewWord(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                __checkTwowayOnly("getNewWord");
                __delBase = __getDelegate(false);
                _WordServerDel __del = (_WordServerDel)__delBase;
                return __del.getNewWord(__ctx);
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

    public static WordServerPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        WordServerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WordServerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::examples::autogen::WordServer"))
                {
                    WordServerPrxHelper __h = new WordServerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WordServerPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        WordServerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WordServerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::examples::autogen::WordServer", __ctx))
                {
                    WordServerPrxHelper __h = new WordServerPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WordServerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WordServerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::examples::autogen::WordServer"))
                {
                    WordServerPrxHelper __h = new WordServerPrxHelper();
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

    public static WordServerPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        WordServerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::examples::autogen::WordServer", __ctx))
                {
                    WordServerPrxHelper __h = new WordServerPrxHelper();
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

    public static WordServerPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        WordServerPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WordServerPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                WordServerPrxHelper __h = new WordServerPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static WordServerPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WordServerPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            WordServerPrxHelper __h = new WordServerPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _WordServerDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _WordServerDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, WordServerPrx v)
    {
        __os.writeProxy(v);
    }

    public static WordServerPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            WordServerPrxHelper result = new WordServerPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }

    public static void
    write(Ice.OutputStream __outS, WordServerPrx v)
    {
        __outS.writeProxy(v);
    }

    public static WordServerPrx
    read(Ice.InputStream __inS)
    {
        Ice.ObjectPrx proxy = __inS.readProxy();
        if(proxy != null)
        {
            WordServerPrxHelper result = new WordServerPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
