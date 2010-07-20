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

public final class ProbDistributionPrxHelper extends Ice.ObjectPrxHelperBase implements ProbDistributionPrx
{
    public static ProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ProbDistributionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ProbDistributionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::ProbDistribution"))
                {
                    ProbDistributionPrxHelper __h = new ProbDistributionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ProbDistributionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ProbDistributionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::ProbDistribution", __ctx))
                {
                    ProbDistributionPrxHelper __h = new ProbDistributionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ProbDistributionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::ProbDistribution"))
                {
                    ProbDistributionPrxHelper __h = new ProbDistributionPrxHelper();
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

    public static ProbDistributionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ProbDistributionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::ProbDistribution", __ctx))
                {
                    ProbDistributionPrxHelper __h = new ProbDistributionPrxHelper();
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

    public static ProbDistributionPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ProbDistributionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ProbDistributionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ProbDistributionPrxHelper __h = new ProbDistributionPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ProbDistributionPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ProbDistributionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ProbDistributionPrxHelper __h = new ProbDistributionPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ProbDistributionDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ProbDistributionDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ProbDistributionPrx v)
    {
        __os.writeProxy(v);
    }

    public static ProbDistributionPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ProbDistributionPrxHelper result = new ProbDistributionPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
