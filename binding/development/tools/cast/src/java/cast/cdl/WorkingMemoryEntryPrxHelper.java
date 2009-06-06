// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.cdl;

public final class WorkingMemoryEntryPrxHelper extends Ice.ObjectPrxHelperBase implements WorkingMemoryEntryPrx
{
    public static WorkingMemoryEntryPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        WorkingMemoryEntryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WorkingMemoryEntryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::WorkingMemoryEntry"))
                {
                    WorkingMemoryEntryPrxHelper __h = new WorkingMemoryEntryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WorkingMemoryEntryPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        WorkingMemoryEntryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WorkingMemoryEntryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::WorkingMemoryEntry", __ctx))
                {
                    WorkingMemoryEntryPrxHelper __h = new WorkingMemoryEntryPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static WorkingMemoryEntryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WorkingMemoryEntryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::WorkingMemoryEntry"))
                {
                    WorkingMemoryEntryPrxHelper __h = new WorkingMemoryEntryPrxHelper();
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

    public static WorkingMemoryEntryPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        WorkingMemoryEntryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::WorkingMemoryEntry", __ctx))
                {
                    WorkingMemoryEntryPrxHelper __h = new WorkingMemoryEntryPrxHelper();
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

    public static WorkingMemoryEntryPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        WorkingMemoryEntryPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (WorkingMemoryEntryPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                WorkingMemoryEntryPrxHelper __h = new WorkingMemoryEntryPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static WorkingMemoryEntryPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        WorkingMemoryEntryPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            WorkingMemoryEntryPrxHelper __h = new WorkingMemoryEntryPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _WorkingMemoryEntryDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _WorkingMemoryEntryDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, WorkingMemoryEntryPrx v)
    {
        __os.writeProxy(v);
    }

    public static WorkingMemoryEntryPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            WorkingMemoryEntryPrxHelper result = new WorkingMemoryEntryPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
