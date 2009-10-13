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

public final class GroundPrxHelper extends Ice.ObjectPrxHelperBase implements GroundPrx
{
    public static GroundPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        GroundPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GroundPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::Ground"))
                {
                    GroundPrxHelper __h = new GroundPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GroundPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        GroundPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GroundPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::Ground", __ctx))
                {
                    GroundPrxHelper __h = new GroundPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GroundPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GroundPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::Ground"))
                {
                    GroundPrxHelper __h = new GroundPrxHelper();
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

    public static GroundPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        GroundPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::Ground", __ctx))
                {
                    GroundPrxHelper __h = new GroundPrxHelper();
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

    public static GroundPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        GroundPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GroundPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                GroundPrxHelper __h = new GroundPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static GroundPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GroundPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            GroundPrxHelper __h = new GroundPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _GroundDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _GroundDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, GroundPrx v)
    {
        __os.writeProxy(v);
    }

    public static GroundPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            GroundPrxHelper result = new GroundPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
