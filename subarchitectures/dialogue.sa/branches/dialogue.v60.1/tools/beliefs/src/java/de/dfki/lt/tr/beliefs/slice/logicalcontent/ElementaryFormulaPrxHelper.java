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

public final class ElementaryFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements ElementaryFormulaPrx
{
    public static ElementaryFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ElementaryFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ElementaryFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ElementaryFormula"))
                {
                    ElementaryFormulaPrxHelper __h = new ElementaryFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ElementaryFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ElementaryFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ElementaryFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ElementaryFormula", __ctx))
                {
                    ElementaryFormulaPrxHelper __h = new ElementaryFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ElementaryFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ElementaryFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ElementaryFormula"))
                {
                    ElementaryFormulaPrxHelper __h = new ElementaryFormulaPrxHelper();
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

    public static ElementaryFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ElementaryFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ElementaryFormula", __ctx))
                {
                    ElementaryFormulaPrxHelper __h = new ElementaryFormulaPrxHelper();
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

    public static ElementaryFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ElementaryFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ElementaryFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ElementaryFormulaPrxHelper __h = new ElementaryFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ElementaryFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ElementaryFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ElementaryFormulaPrxHelper __h = new ElementaryFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ElementaryFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ElementaryFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ElementaryFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static ElementaryFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ElementaryFormulaPrxHelper result = new ElementaryFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
