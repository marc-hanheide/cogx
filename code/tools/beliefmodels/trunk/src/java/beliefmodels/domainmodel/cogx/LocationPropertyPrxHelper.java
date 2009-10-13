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

public final class LocationPropertyPrxHelper extends Ice.ObjectPrxHelperBase implements LocationPropertyPrx
{
    public static LocationPropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        LocationPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LocationPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LocationProperty"))
                {
                    LocationPropertyPrxHelper __h = new LocationPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LocationPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        LocationPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LocationPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LocationProperty", __ctx))
                {
                    LocationPropertyPrxHelper __h = new LocationPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LocationPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LocationPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LocationProperty"))
                {
                    LocationPropertyPrxHelper __h = new LocationPropertyPrxHelper();
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

    public static LocationPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        LocationPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LocationProperty", __ctx))
                {
                    LocationPropertyPrxHelper __h = new LocationPropertyPrxHelper();
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

    public static LocationPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        LocationPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LocationPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                LocationPropertyPrxHelper __h = new LocationPropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static LocationPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LocationPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            LocationPropertyPrxHelper __h = new LocationPropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _LocationPropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _LocationPropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, LocationPropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static LocationPropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            LocationPropertyPrxHelper result = new LocationPropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
