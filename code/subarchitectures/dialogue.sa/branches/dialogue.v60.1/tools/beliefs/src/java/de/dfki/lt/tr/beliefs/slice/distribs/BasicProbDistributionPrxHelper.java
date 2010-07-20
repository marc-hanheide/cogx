// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.distribs;

public final class BasicProbDistributionPrxHelper extends Ice.ObjectPrxHelperBase implements BasicProbDistributionPrx
{
    public static BasicProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        BasicProbDistributionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BasicProbDistributionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::BasicProbDistribution"))
                {
                    BasicProbDistributionPrxHelper __h = new BasicProbDistributionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BasicProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        BasicProbDistributionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BasicProbDistributionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::BasicProbDistribution", __ctx))
                {
                    BasicProbDistributionPrxHelper __h = new BasicProbDistributionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BasicProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BasicProbDistributionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::BasicProbDistribution"))
                {
                    BasicProbDistributionPrxHelper __h = new BasicProbDistributionPrxHelper();
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

    public static BasicProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        BasicProbDistributionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::BasicProbDistribution", __ctx))
                {
                    BasicProbDistributionPrxHelper __h = new BasicProbDistributionPrxHelper();
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

    public static BasicProbDistributionPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        BasicProbDistributionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BasicProbDistributionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                BasicProbDistributionPrxHelper __h = new BasicProbDistributionPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static BasicProbDistributionPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BasicProbDistributionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            BasicProbDistributionPrxHelper __h = new BasicProbDistributionPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _BasicProbDistributionDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _BasicProbDistributionDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, BasicProbDistributionPrx v)
    {
        __os.writeProxy(v);
    }

    public static BasicProbDistributionPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            BasicProbDistributionPrxHelper result = new BasicProbDistributionPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
