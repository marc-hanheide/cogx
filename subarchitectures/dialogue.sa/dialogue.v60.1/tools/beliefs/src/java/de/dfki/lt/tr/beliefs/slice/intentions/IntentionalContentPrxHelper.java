// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.intentions;

public final class IntentionalContentPrxHelper extends Ice.ObjectPrxHelperBase implements IntentionalContentPrx
{
    public static IntentionalContentPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        IntentionalContentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntentionalContentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent"))
                {
                    IntentionalContentPrxHelper __h = new IntentionalContentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static IntentionalContentPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        IntentionalContentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntentionalContentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent", __ctx))
                {
                    IntentionalContentPrxHelper __h = new IntentionalContentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static IntentionalContentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        IntentionalContentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent"))
                {
                    IntentionalContentPrxHelper __h = new IntentionalContentPrxHelper();
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

    public static IntentionalContentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        IntentionalContentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent", __ctx))
                {
                    IntentionalContentPrxHelper __h = new IntentionalContentPrxHelper();
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

    public static IntentionalContentPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        IntentionalContentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntentionalContentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                IntentionalContentPrxHelper __h = new IntentionalContentPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static IntentionalContentPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        IntentionalContentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            IntentionalContentPrxHelper __h = new IntentionalContentPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _IntentionalContentDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _IntentionalContentDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, IntentionalContentPrx v)
    {
        __os.writeProxy(v);
    }

    public static IntentionalContentPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            IntentionalContentPrxHelper result = new IntentionalContentPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
