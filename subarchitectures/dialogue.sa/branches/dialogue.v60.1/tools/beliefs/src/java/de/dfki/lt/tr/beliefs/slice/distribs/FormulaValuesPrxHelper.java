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

public final class FormulaValuesPrxHelper extends Ice.ObjectPrxHelperBase implements FormulaValuesPrx
{
    public static FormulaValuesPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        FormulaValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FormulaValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::FormulaValues"))
                {
                    FormulaValuesPrxHelper __h = new FormulaValuesPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static FormulaValuesPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        FormulaValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FormulaValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::FormulaValues", __ctx))
                {
                    FormulaValuesPrxHelper __h = new FormulaValuesPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static FormulaValuesPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        FormulaValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::FormulaValues"))
                {
                    FormulaValuesPrxHelper __h = new FormulaValuesPrxHelper();
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

    public static FormulaValuesPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        FormulaValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::distribs::FormulaValues", __ctx))
                {
                    FormulaValuesPrxHelper __h = new FormulaValuesPrxHelper();
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

    public static FormulaValuesPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        FormulaValuesPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (FormulaValuesPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                FormulaValuesPrxHelper __h = new FormulaValuesPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static FormulaValuesPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        FormulaValuesPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            FormulaValuesPrxHelper __h = new FormulaValuesPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _FormulaValuesDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _FormulaValuesDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, FormulaValuesPrx v)
    {
        __os.writeProxy(v);
    }

    public static FormulaValuesPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            FormulaValuesPrxHelper result = new FormulaValuesPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
