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

public final class RelationArgumentPrxHelper extends Ice.ObjectPrxHelperBase implements RelationArgumentPrx
{
    public static RelationArgumentPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        RelationArgumentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (RelationArgumentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::RelationArgument"))
                {
                    RelationArgumentPrxHelper __h = new RelationArgumentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static RelationArgumentPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        RelationArgumentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (RelationArgumentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::RelationArgument", __ctx))
                {
                    RelationArgumentPrxHelper __h = new RelationArgumentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static RelationArgumentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        RelationArgumentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::RelationArgument"))
                {
                    RelationArgumentPrxHelper __h = new RelationArgumentPrxHelper();
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

    public static RelationArgumentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        RelationArgumentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::RelationArgument", __ctx))
                {
                    RelationArgumentPrxHelper __h = new RelationArgumentPrxHelper();
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

    public static RelationArgumentPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        RelationArgumentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (RelationArgumentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                RelationArgumentPrxHelper __h = new RelationArgumentPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static RelationArgumentPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        RelationArgumentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            RelationArgumentPrxHelper __h = new RelationArgumentPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _RelationArgumentDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _RelationArgumentDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, RelationArgumentPrx v)
    {
        __os.writeProxy(v);
    }

    public static RelationArgumentPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            RelationArgumentPrxHelper result = new RelationArgumentPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
