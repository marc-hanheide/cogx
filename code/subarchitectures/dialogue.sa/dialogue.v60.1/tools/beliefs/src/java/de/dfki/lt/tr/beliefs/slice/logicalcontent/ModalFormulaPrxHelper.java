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

public final class ModalFormulaPrxHelper extends Ice.ObjectPrxHelperBase implements ModalFormulaPrx
{
    public static ModalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ModalFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ModalFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ModalFormula"))
                {
                    ModalFormulaPrxHelper __h = new ModalFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ModalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ModalFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ModalFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ModalFormula", __ctx))
                {
                    ModalFormulaPrxHelper __h = new ModalFormulaPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ModalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ModalFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ModalFormula"))
                {
                    ModalFormulaPrxHelper __h = new ModalFormulaPrxHelper();
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

    public static ModalFormulaPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ModalFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::de::dfki::lt::tr::beliefs::slice::logicalcontent::ModalFormula", __ctx))
                {
                    ModalFormulaPrxHelper __h = new ModalFormulaPrxHelper();
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

    public static ModalFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ModalFormulaPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ModalFormulaPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ModalFormulaPrxHelper __h = new ModalFormulaPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ModalFormulaPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ModalFormulaPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ModalFormulaPrxHelper __h = new ModalFormulaPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ModalFormulaDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ModalFormulaDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ModalFormulaPrx v)
    {
        __os.writeProxy(v);
    }

    public static ModalFormulaPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ModalFormulaPrxHelper result = new ModalFormulaPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
