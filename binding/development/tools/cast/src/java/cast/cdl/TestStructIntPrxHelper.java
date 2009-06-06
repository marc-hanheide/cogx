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

public final class TestStructIntPrxHelper extends Ice.ObjectPrxHelperBase implements TestStructIntPrx
{
    public static TestStructIntPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TestStructIntPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestStructIntPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::TestStructInt"))
                {
                    TestStructIntPrxHelper __h = new TestStructIntPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TestStructIntPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TestStructIntPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestStructIntPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::TestStructInt", __ctx))
                {
                    TestStructIntPrxHelper __h = new TestStructIntPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TestStructIntPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TestStructIntPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::TestStructInt"))
                {
                    TestStructIntPrxHelper __h = new TestStructIntPrxHelper();
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

    public static TestStructIntPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TestStructIntPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::TestStructInt", __ctx))
                {
                    TestStructIntPrxHelper __h = new TestStructIntPrxHelper();
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

    public static TestStructIntPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TestStructIntPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestStructIntPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TestStructIntPrxHelper __h = new TestStructIntPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TestStructIntPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TestStructIntPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TestStructIntPrxHelper __h = new TestStructIntPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TestStructIntDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TestStructIntDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TestStructIntPrx v)
    {
        __os.writeProxy(v);
    }

    public static TestStructIntPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TestStructIntPrxHelper result = new TestStructIntPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
