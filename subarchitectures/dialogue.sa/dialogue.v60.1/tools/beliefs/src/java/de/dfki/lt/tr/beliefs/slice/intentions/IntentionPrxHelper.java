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

public final class IntentionPrxHelper extends Ice.ObjectPrxHelperBase implements IntentionPrx
{
    public static IntentionPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        IntentionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntentionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::Intention"))
                {
                    IntentionPrxHelper __h = new IntentionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static IntentionPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        IntentionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntentionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::Intention", __ctx))
                {
                    IntentionPrxHelper __h = new IntentionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static IntentionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        IntentionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::Intention"))
                {
                    IntentionPrxHelper __h = new IntentionPrxHelper();
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

    public static IntentionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        IntentionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::intentions::Intention", __ctx))
                {
                    IntentionPrxHelper __h = new IntentionPrxHelper();
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

    public static IntentionPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        IntentionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntentionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                IntentionPrxHelper __h = new IntentionPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static IntentionPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        IntentionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            IntentionPrxHelper __h = new IntentionPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _IntentionDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _IntentionDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, IntentionPrx v)
    {
        __os.writeProxy(v);
    }

    public static IntentionPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            IntentionPrxHelper result = new IntentionPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
