// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.domainmodel.cogx;

public final class LinguisticAttributePropertyPrxHelper extends Ice.ObjectPrxHelperBase implements LinguisticAttributePropertyPrx
{
    public static LinguisticAttributePropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        LinguisticAttributePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LinguisticAttributePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticAttributeProperty"))
                {
                    LinguisticAttributePropertyPrxHelper __h = new LinguisticAttributePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LinguisticAttributePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        LinguisticAttributePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LinguisticAttributePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticAttributeProperty", __ctx))
                {
                    LinguisticAttributePropertyPrxHelper __h = new LinguisticAttributePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LinguisticAttributePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LinguisticAttributePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticAttributeProperty"))
                {
                    LinguisticAttributePropertyPrxHelper __h = new LinguisticAttributePropertyPrxHelper();
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

    public static LinguisticAttributePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        LinguisticAttributePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticAttributeProperty", __ctx))
                {
                    LinguisticAttributePropertyPrxHelper __h = new LinguisticAttributePropertyPrxHelper();
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

    public static LinguisticAttributePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        LinguisticAttributePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LinguisticAttributePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                LinguisticAttributePropertyPrxHelper __h = new LinguisticAttributePropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static LinguisticAttributePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LinguisticAttributePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            LinguisticAttributePropertyPrxHelper __h = new LinguisticAttributePropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _LinguisticAttributePropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _LinguisticAttributePropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, LinguisticAttributePropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static LinguisticAttributePropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            LinguisticAttributePropertyPrxHelper result = new LinguisticAttributePropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
