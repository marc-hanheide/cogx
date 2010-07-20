// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.logicalcontent;

public final class UnknownFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements UnknownFormulaPrx
{
    public static UnknownFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        UnknownFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnknownFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnknownFormula"))
                {
                    UnknownFormulaPrxHelper __h = new UnknownFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UnknownFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        UnknownFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnknownFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnknownFormula", __ctx))
                {
                    UnknownFormulaPrxHelper __h = new UnknownFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static UnknownFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UnknownFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnknownFormula"))
                {
                    UnknownFormulaPrxHelper __h = new UnknownFormulaPrxHelper();
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

    public static UnknownFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        UnknownFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::UnknownFormula", __ctx))
                {
                    UnknownFormulaPrxHelper __h = new UnknownFormulaPrxHelper();
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

    public static UnknownFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        UnknownFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (UnknownFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                UnknownFormulaPrxHelper __h = new UnknownFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static UnknownFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        UnknownFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            UnknownFormulaPrxHelper __h = new UnknownFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _UnknownFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _UnknownFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, UnknownFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static UnknownFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            UnknownFormulaPrxHelper result = new UnknownFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
