// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.adl;

public final class TaskPrxHelper extends Ice.ObjectPrxHelperBase implements TaskPrx
{
    public static TaskPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Task"))
                {
                    TaskPrxHelper __h = new TaskPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TaskPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Task", __ctx))
                {
                    TaskPrxHelper __h = new TaskPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TaskPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Task"))
                {
                    TaskPrxHelper __h = new TaskPrxHelper();
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

    public static TaskPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Task", __ctx))
                {
                    TaskPrxHelper __h = new TaskPrxHelper();
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

    public static TaskPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TaskPrxHelper __h = new TaskPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TaskPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TaskPrxHelper __h = new TaskPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TaskDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TaskDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TaskPrx v)
    {
        __os.writeProxy(v);
    }

    public static TaskPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TaskPrxHelper result = new TaskPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
