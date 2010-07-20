// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.sitbeliefs;

public final class dBeliefPrxHelper extends Ice.ObjectPrxHelperBase implements dBeliefPrx
{
    public static dBeliefPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        dBeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (dBeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief"))
                {
                    dBeliefPrxHelper __h = new dBeliefPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static dBeliefPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        dBeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (dBeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief", __ctx))
                {
                    dBeliefPrxHelper __h = new dBeliefPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static dBeliefPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        dBeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief"))
                {
                    dBeliefPrxHelper __h = new dBeliefPrxHelper();
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

    public static dBeliefPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        dBeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief", __ctx))
                {
                    dBeliefPrxHelper __h = new dBeliefPrxHelper();
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

    public static dBeliefPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        dBeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (dBeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                dBeliefPrxHelper __h = new dBeliefPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static dBeliefPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        dBeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            dBeliefPrxHelper __h = new dBeliefPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _dBeliefDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _dBeliefDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, dBeliefPrx v)
    {
        __os.writeProxy(v);
    }

    public static dBeliefPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            dBeliefPrxHelper result = new dBeliefPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
