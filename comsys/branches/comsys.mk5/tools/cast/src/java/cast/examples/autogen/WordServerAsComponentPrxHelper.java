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

public final class WordServerAsComponentPrxHelper extends Ice.ObjectPrxHelperBase implements WordServerAsComponentPrx
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
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

    public void
    beat()
    {
        beat(null, false);
    }

    public void
    beat(java.util.Map<String, String> __ctx)
    {
        beat(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    beat(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.beat(__ctx);
                return;
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    configure(java.util.Map<java.lang.String, java.lang.String> config)
    {
        configure(config, null, false);
    }

    public void
    configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx)
    {
        configure(config, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.configure(config, __ctx);
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

    public void
    destroy()
    {
        destroy(null, false);
    }

    public void
    destroy(java.util.Map<String, String> __ctx)
    {
        destroy(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    destroy(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.destroy(__ctx);
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

    public String
    getID()
    {
        return getID(null, false);
    }

    public String
    getID(java.util.Map<String, String> __ctx)
    {
        return getID(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private String
    getID(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                __checkTwowayOnly("getID");
                __delBase = __getDelegate(false);
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                return __del.getID(__ctx);
            }
            catch(IceInternal.LocalExceptionWrapper __ex)
            {
                __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt);
            }
            catch(Ice.LocalException __ex)
            {
                __cnt = __handleException(__delBase, __ex, null, __cnt);
            }
        }
    }

    public void
    run()
    {
        run(null, false);
    }

    public void
    run(java.util.Map<String, String> __ctx)
    {
        run(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    run(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.run(__ctx);
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

    public void
    setComponentManager(cast.interfaces.ComponentManagerPrx man)
    {
        setComponentManager(man, null, false);
    }

    public void
    setComponentManager(cast.interfaces.ComponentManagerPrx man, java.util.Map<String, String> __ctx)
    {
        setComponentManager(man, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setComponentManager(cast.interfaces.ComponentManagerPrx man, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.setComponentManager(man, __ctx);
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

    public void
    setID(String id)
    {
        setID(id, null, false);
    }

    public void
    setID(String id, java.util.Map<String, String> __ctx)
    {
        setID(id, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setID(String id, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.setID(id, __ctx);
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

    public void
    setTimeServer(cast.interfaces.TimeServerPrx ts)
    {
        setTimeServer(ts, null, false);
    }

    public void
    setTimeServer(cast.interfaces.TimeServerPrx ts, java.util.Map<String, String> __ctx)
    {
        setTimeServer(ts, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    setTimeServer(cast.interfaces.TimeServerPrx ts, java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.setTimeServer(ts, __ctx);
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

    public void
    start()
    {
        start(null, false);
    }

    public void
    start(java.util.Map<String, String> __ctx)
    {
        start(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    start(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.start(__ctx);
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

    public void
    stop()
    {
        stop(null, false);
    }

    public void
    stop(java.util.Map<String, String> __ctx)
    {
        stop(__ctx, true);
    }

    @SuppressWarnings("unchecked")
    private void
    stop(java.util.Map<String, String> __ctx, boolean __explicitCtx)
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
                _WordServerAsComponentDel __del = (_WordServerAsComponentDel)__delBase;
                __del.stop(__ctx);
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

    public static WordServerAsComponentPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        WordServerAsComponentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WordServerAsComponentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::examples::autogen::WordServerAsComponent"))
                {
                    WordServerAsComponentPrxHelper __h = new WordServerAsComponentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WordServerAsComponentPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        WordServerAsComponentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WordServerAsComponentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::examples::autogen::WordServerAsComponent", __ctx))
                {
                    WordServerAsComponentPrxHelper __h = new WordServerAsComponentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WordServerAsComponentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WordServerAsComponentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::examples::autogen::WordServerAsComponent"))
                {
                    WordServerAsComponentPrxHelper __h = new WordServerAsComponentPrxHelper();
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

    public static WordServerAsComponentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        WordServerAsComponentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::examples::autogen::WordServerAsComponent", __ctx))
                {
                    WordServerAsComponentPrxHelper __h = new WordServerAsComponentPrxHelper();
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

    public static WordServerAsComponentPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        WordServerAsComponentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WordServerAsComponentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                WordServerAsComponentPrxHelper __h = new WordServerAsComponentPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static WordServerAsComponentPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WordServerAsComponentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            WordServerAsComponentPrxHelper __h = new WordServerAsComponentPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _WordServerAsComponentDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _WordServerAsComponentDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, WordServerAsComponentPrx v)
    {
        __os.writeProxy(v);
    }

    public static WordServerAsComponentPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            WordServerAsComponentPrxHelper result = new WordServerAsComponentPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }

    public static void
    write(Ice.OutputStream __outS, WordServerAsComponentPrx v)
    {
        __outS.writeProxy(v);
    }

    public static WordServerAsComponentPrx
    read(Ice.InputStream __inS)
    {
        Ice.ObjectPrx proxy = __inS.readProxy();
        if(proxy != null)
        {
            WordServerAsComponentPrxHelper result = new WordServerAsComponentPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
