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

public final class IntegerFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements IntegerFormulaPrx
{
    public static IntegerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        IntegerFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntegerFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::IntegerFormula"))
                {
                    IntegerFormulaPrxHelper __h = new IntegerFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static IntegerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        IntegerFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntegerFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::IntegerFormula", __ctx))
                {
                    IntegerFormulaPrxHelper __h = new IntegerFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static IntegerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        IntegerFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::IntegerFormula"))
                {
                    IntegerFormulaPrxHelper __h = new IntegerFormulaPrxHelper();
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

    public static IntegerFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        IntegerFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::IntegerFormula", __ctx))
                {
                    IntegerFormulaPrxHelper __h = new IntegerFormulaPrxHelper();
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

    public static IntegerFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        IntegerFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (IntegerFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                IntegerFormulaPrxHelper __h = new IntegerFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static IntegerFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        IntegerFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            IntegerFormulaPrxHelper __h = new IntegerFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _IntegerFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _IntegerFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, IntegerFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static IntegerFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            IntegerFormulaPrxHelper result = new IntegerFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
