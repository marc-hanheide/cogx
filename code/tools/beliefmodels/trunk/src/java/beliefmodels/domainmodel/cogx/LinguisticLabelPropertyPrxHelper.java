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

public final class LinguisticLabelPropertyPrxHelper extends Ice.ObjectPrxHelperBase implements LinguisticLabelPropertyPrx
{
    public static LinguisticLabelPropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        LinguisticLabelPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LinguisticLabelPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticLabelProperty"))
                {
                    LinguisticLabelPropertyPrxHelper __h = new LinguisticLabelPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LinguisticLabelPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        LinguisticLabelPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LinguisticLabelPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticLabelProperty", __ctx))
                {
                    LinguisticLabelPropertyPrxHelper __h = new LinguisticLabelPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static LinguisticLabelPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LinguisticLabelPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticLabelProperty"))
                {
                    LinguisticLabelPropertyPrxHelper __h = new LinguisticLabelPropertyPrxHelper();
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

    public static LinguisticLabelPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        LinguisticLabelPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::LinguisticLabelProperty", __ctx))
                {
                    LinguisticLabelPropertyPrxHelper __h = new LinguisticLabelPropertyPrxHelper();
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

    public static LinguisticLabelPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        LinguisticLabelPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (LinguisticLabelPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                LinguisticLabelPropertyPrxHelper __h = new LinguisticLabelPropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static LinguisticLabelPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        LinguisticLabelPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            LinguisticLabelPropertyPrxHelper __h = new LinguisticLabelPropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _LinguisticLabelPropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _LinguisticLabelPropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, LinguisticLabelPropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static LinguisticLabelPropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            LinguisticLabelPropertyPrxHelper result = new LinguisticLabelPropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
