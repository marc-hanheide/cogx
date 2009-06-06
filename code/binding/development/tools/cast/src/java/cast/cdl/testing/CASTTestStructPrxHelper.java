// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.cdl.testing;

public final class CASTTestStructPrxHelper extends Ice.ObjectPrxHelperBase implements CASTTestStructPrx
{
    public static CASTTestStructPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        CASTTestStructPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (CASTTestStructPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::testing::CASTTestStruct"))
                {
                    CASTTestStructPrxHelper __h = new CASTTestStructPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static CASTTestStructPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        CASTTestStructPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (CASTTestStructPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::testing::CASTTestStruct", __ctx))
                {
                    CASTTestStructPrxHelper __h = new CASTTestStructPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static CASTTestStructPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        CASTTestStructPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::testing::CASTTestStruct"))
                {
                    CASTTestStructPrxHelper __h = new CASTTestStructPrxHelper();
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

    public static CASTTestStructPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        CASTTestStructPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::testing::CASTTestStruct", __ctx))
                {
                    CASTTestStructPrxHelper __h = new CASTTestStructPrxHelper();
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

    public static CASTTestStructPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        CASTTestStructPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (CASTTestStructPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                CASTTestStructPrxHelper __h = new CASTTestStructPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static CASTTestStructPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        CASTTestStructPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            CASTTestStructPrxHelper __h = new CASTTestStructPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _CASTTestStructDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _CASTTestStructDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, CASTTestStructPrx v)
    {
        __os.writeProxy(v);
    }

    public static CASTTestStructPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            CASTTestStructPrxHelper result = new CASTTestStructPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
