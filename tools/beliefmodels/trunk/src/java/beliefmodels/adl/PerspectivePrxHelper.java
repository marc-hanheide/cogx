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

public final class PerspectivePrxHelper extends Ice.ObjectPrxHelperBase implements PerspectivePrx
{
    public static PerspectivePrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        PerspectivePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PerspectivePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Perspective"))
                {
                    PerspectivePrxHelper __h = new PerspectivePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static PerspectivePrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        PerspectivePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PerspectivePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Perspective", __ctx))
                {
                    PerspectivePrxHelper __h = new PerspectivePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static PerspectivePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        PerspectivePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Perspective"))
                {
                    PerspectivePrxHelper __h = new PerspectivePrxHelper();
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

    public static PerspectivePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        PerspectivePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Perspective", __ctx))
                {
                    PerspectivePrxHelper __h = new PerspectivePrxHelper();
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

    public static PerspectivePrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        PerspectivePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PerspectivePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                PerspectivePrxHelper __h = new PerspectivePrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static PerspectivePrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        PerspectivePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            PerspectivePrxHelper __h = new PerspectivePrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _PerspectiveDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _PerspectiveDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, PerspectivePrx v)
    {
        __os.writeProxy(v);
    }

    public static PerspectivePrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            PerspectivePrxHelper result = new PerspectivePrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
