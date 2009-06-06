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

public final class TestDummyStructPrxHelper extends Ice.ObjectPrxHelperBase implements TestDummyStructPrx
{
    public static TestDummyStructPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TestDummyStructPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestDummyStructPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::testing::TestDummyStruct"))
                {
                    TestDummyStructPrxHelper __h = new TestDummyStructPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TestDummyStructPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TestDummyStructPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestDummyStructPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::testing::TestDummyStruct", __ctx))
                {
                    TestDummyStructPrxHelper __h = new TestDummyStructPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TestDummyStructPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TestDummyStructPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::testing::TestDummyStruct"))
                {
                    TestDummyStructPrxHelper __h = new TestDummyStructPrxHelper();
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

    public static TestDummyStructPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TestDummyStructPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::testing::TestDummyStruct", __ctx))
                {
                    TestDummyStructPrxHelper __h = new TestDummyStructPrxHelper();
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

    public static TestDummyStructPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TestDummyStructPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestDummyStructPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TestDummyStructPrxHelper __h = new TestDummyStructPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TestDummyStructPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TestDummyStructPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TestDummyStructPrxHelper __h = new TestDummyStructPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TestDummyStructDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TestDummyStructDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TestDummyStructPrx v)
    {
        __os.writeProxy(v);
    }

    public static TestDummyStructPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TestDummyStructPrxHelper result = new TestDummyStructPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
