// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.domainmodel.cogx;

public final class UnionRefPropertyPrxHelper extends Ice.ObjectPrxHelperBase implements UnionRefPropertyPrx
{
    public static UnionRefPropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        UnionRefPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnionRefPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::UnionRefProperty"))
                {
                    UnionRefPropertyPrxHelper __h = new UnionRefPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UnionRefPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        UnionRefPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnionRefPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::UnionRefProperty", __ctx))
                {
                    UnionRefPropertyPrxHelper __h = new UnionRefPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UnionRefPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UnionRefPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::UnionRefProperty"))
                {
                    UnionRefPropertyPrxHelper __h = new UnionRefPropertyPrxHelper();
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

    public static UnionRefPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        UnionRefPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::UnionRefProperty", __ctx))
                {
                    UnionRefPropertyPrxHelper __h = new UnionRefPropertyPrxHelper();
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

    public static UnionRefPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        UnionRefPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnionRefPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                UnionRefPropertyPrxHelper __h = new UnionRefPropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static UnionRefPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UnionRefPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            UnionRefPropertyPrxHelper __h = new UnionRefPropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _UnionRefPropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _UnionRefPropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, UnionRefPropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static UnionRefPropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            UnionRefPropertyPrxHelper result = new UnionRefPropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
