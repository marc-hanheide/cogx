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

public final class TestStructStringPrxHelper extends Ice.ObjectPrxHelperBase implements TestStructStringPrx
{
    public static TestStructStringPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        TestStructStringPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestStructStringPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::TestStructString"))
                {
                    TestStructStringPrxHelper __h = new TestStructStringPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TestStructStringPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        TestStructStringPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestStructStringPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::cast::cdl::TestStructString", __ctx))
                {
                    TestStructStringPrxHelper __h = new TestStructStringPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static TestStructStringPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TestStructStringPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::TestStructString"))
                {
                    TestStructStringPrxHelper __h = new TestStructStringPrxHelper();
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

    public static TestStructStringPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        TestStructStringPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::cast::cdl::TestStructString", __ctx))
                {
                    TestStructStringPrxHelper __h = new TestStructStringPrxHelper();
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

    public static TestStructStringPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        TestStructStringPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (TestStructStringPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                TestStructStringPrxHelper __h = new TestStructStringPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static TestStructStringPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        TestStructStringPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            TestStructStringPrxHelper __h = new TestStructStringPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _TestStructStringDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _TestStructStringDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, TestStructStringPrx v)
    {
        __os.writeProxy(v);
    }

    public static TestStructStringPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            TestStructStringPrxHelper result = new TestStructStringPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
