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

public final class GenericPointerFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements GenericPointerFormulaPrx
{
    public static GenericPointerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        GenericPointerFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GenericPointerFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::GenericPointerFormula"))
                {
                    GenericPointerFormulaPrxHelper __h = new GenericPointerFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GenericPointerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        GenericPointerFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GenericPointerFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::GenericPointerFormula", __ctx))
                {
                    GenericPointerFormulaPrxHelper __h = new GenericPointerFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GenericPointerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GenericPointerFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::GenericPointerFormula"))
                {
                    GenericPointerFormulaPrxHelper __h = new GenericPointerFormulaPrxHelper();
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

    public static GenericPointerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        GenericPointerFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::GenericPointerFormula", __ctx))
                {
                    GenericPointerFormulaPrxHelper __h = new GenericPointerFormulaPrxHelper();
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

    public static GenericPointerFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        GenericPointerFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GenericPointerFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                GenericPointerFormulaPrxHelper __h = new GenericPointerFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static GenericPointerFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GenericPointerFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            GenericPointerFormulaPrxHelper __h = new GenericPointerFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _GenericPointerFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _GenericPointerFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, GenericPointerFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static GenericPointerFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            GenericPointerFormulaPrxHelper result = new GenericPointerFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
