// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0

package cast.interfaces;

public final class ComponentFactoryPrxHelper extends Ice.ObjectPrxHelperBase implements ComponentFactoryPrx
{
    public CASTComponentPrx
    newComponent(String id, String type)
        throws cast.ComponentCreationException
    {
        return newComponent(id, type, null, false);
    }

    public CASTComponentPrx
    newComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException
    {
        return newComponent(id, type, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private CASTComponentPrx
    newComponent(String id, String type, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.ComponentCreationException
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
                __checkTwowayOnly("newComponent");
                __delBase = __getDelegate(false);
                _ComponentFactoryDel __del = (_ComponentFactoryDel)__delBase;
                return __del.newComponent(id, type, __ctx);
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

    public ManagedComponentPrx
    newManagedComponent(String id, String type)
        throws cast.ComponentCreationException
    {
        return newManagedComponent(id, type, null, false);
    }

    public ManagedComponentPrx
    newManagedComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException
    {
        return newManagedComponent(id, type, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private ManagedComponentPrx
    newManagedComponent(String id, String type, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.ComponentCreationException
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
                __checkTwowayOnly("newManagedComponent");
                __delBase = __getDelegate(false);
                _ComponentFactoryDel __del = (_ComponentFactoryDel)__delBase;
                return __del.newManagedComponent(id, type, __ctx);
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

    public TaskManagerPrx
    newTaskManager(String id, String type)
        throws cast.ComponentCreationException
    {
        return newTaskManager(id, type, null, false);
    }

    public TaskManagerPrx
    newTaskManager(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException
    {
        return newTaskManager(id, type, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private TaskManagerPrx
    newTaskManager(String id, String type, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.ComponentCreationException
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
                __checkTwowayOnly("newTaskManager");
                __delBase = __getDelegate(false);
                _ComponentFactoryDel __del = (_ComponentFactoryDel)__delBase;
                return __del.newTaskManager(id, type, __ctx);
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

    public UnmanagedComponentPrx
    newUnmanagedComponent(String id, String type)
        throws cast.ComponentCreationException
    {
        return newUnmanagedComponent(id, type, null, false);
    }

    public UnmanagedComponentPrx
    newUnmanagedComponent(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException
    {
        return newUnmanagedComponent(id, type, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private UnmanagedComponentPrx
    newUnmanagedComponent(String id, String type, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.ComponentCreationException
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
                __checkTwowayOnly("newUnmanagedComponent");
                __delBase = __getDelegate(false);
                _ComponentFactoryDel __del = (_ComponentFactoryDel)__delBase;
                return __del.newUnmanagedComponent(id, type, __ctx);
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

    public WorkingMemoryPrx
    newWorkingMemory(String id, String type)
        throws cast.ComponentCreationException
    {
        return newWorkingMemory(id, type, null, false);
    }

    public WorkingMemoryPrx
    newWorkingMemory(String id, String type, java.util.Map<String, String> __ctx)
        throws cast.ComponentCreationException
    {
        return newWorkingMemory(id, type, __ctx, true);
    }

    @SuppressWarnings("unchecked")
    private WorkingMemoryPrx
    newWorkingMemory(String id, String type, java.util.Map<String, String> __ctx, boolean __explicitCtx)
        throws cast.ComponentCreationException
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
                __checkTwowayOnly("newWorkingMemory");
                __delBase = __getDelegate(false);
                _ComponentFactoryDel __del = (_ComponentFactoryDel)__delBase;
                return __del.newWorkingMemory(id, type, __ctx);
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

    public static ComponentFactoryPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ComponentFactoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComponentFactoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::ComponentFactory"))
                {
                    ComponentFactoryPrxHelper __h = new ComponentFactoryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ComponentFactoryPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ComponentFactoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComponentFactoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::interfaces::ComponentFactory", __ctx))
                {
                    ComponentFactoryPrxHelper __h = new ComponentFactoryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ComponentFactoryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ComponentFactoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::ComponentFactory"))
                {
                    ComponentFactoryPrxHelper __h = new ComponentFactoryPrxHelper();
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

    public static ComponentFactoryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ComponentFactoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::interfaces::ComponentFactory", __ctx))
                {
                    ComponentFactoryPrxHelper __h = new ComponentFactoryPrxHelper();
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

    public static ComponentFactoryPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ComponentFactoryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ComponentFactoryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ComponentFactoryPrxHelper __h = new ComponentFactoryPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ComponentFactoryPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ComponentFactoryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ComponentFactoryPrxHelper __h = new ComponentFactoryPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ComponentFactoryDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ComponentFactoryDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ComponentFactoryPrx v)
    {
        __os.writeProxy(v);
    }

    public static ComponentFactoryPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ComponentFactoryPrxHelper result = new ComponentFactoryPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
