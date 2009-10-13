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

public final class SpatioTemporalModelPrxHelper extends Ice.ObjectPrxHelperBase implements SpatioTemporalModelPrx
{
    public static SpatioTemporalModelPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        SpatioTemporalModelPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SpatioTemporalModelPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::SpatioTemporalModel"))
                {
                    SpatioTemporalModelPrxHelper __h = new SpatioTemporalModelPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SpatioTemporalModelPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        SpatioTemporalModelPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SpatioTemporalModelPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::SpatioTemporalModel", __ctx))
                {
                    SpatioTemporalModelPrxHelper __h = new SpatioTemporalModelPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SpatioTemporalModelPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SpatioTemporalModelPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::SpatioTemporalModel"))
                {
                    SpatioTemporalModelPrxHelper __h = new SpatioTemporalModelPrxHelper();
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

    public static SpatioTemporalModelPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        SpatioTemporalModelPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::SpatioTemporalModel", __ctx))
                {
                    SpatioTemporalModelPrxHelper __h = new SpatioTemporalModelPrxHelper();
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

    public static SpatioTemporalModelPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        SpatioTemporalModelPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SpatioTemporalModelPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                SpatioTemporalModelPrxHelper __h = new SpatioTemporalModelPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static SpatioTemporalModelPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SpatioTemporalModelPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            SpatioTemporalModelPrxHelper __h = new SpatioTemporalModelPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _SpatioTemporalModelDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _SpatioTemporalModelDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, SpatioTemporalModelPrx v)
    {
        __os.writeProxy(v);
    }

    public static SpatioTemporalModelPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            SpatioTemporalModelPrxHelper result = new SpatioTemporalModelPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
