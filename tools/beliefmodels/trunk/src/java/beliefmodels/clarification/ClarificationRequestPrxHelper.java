// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.clarification;

public final class ClarificationRequestPrxHelper extends Ice.ObjectPrxHelperBase implements ClarificationRequestPrx
{
    public static ClarificationRequestPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ClarificationRequestPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ClarificationRequestPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::clarification::ClarificationRequest"))
                {
                    ClarificationRequestPrxHelper __h = new ClarificationRequestPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ClarificationRequestPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ClarificationRequestPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ClarificationRequestPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::clarification::ClarificationRequest", __ctx))
                {
                    ClarificationRequestPrxHelper __h = new ClarificationRequestPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ClarificationRequestPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ClarificationRequestPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::clarification::ClarificationRequest"))
                {
                    ClarificationRequestPrxHelper __h = new ClarificationRequestPrxHelper();
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

    public static ClarificationRequestPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ClarificationRequestPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::clarification::ClarificationRequest", __ctx))
                {
                    ClarificationRequestPrxHelper __h = new ClarificationRequestPrxHelper();
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

    public static ClarificationRequestPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ClarificationRequestPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ClarificationRequestPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ClarificationRequestPrxHelper __h = new ClarificationRequestPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ClarificationRequestPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ClarificationRequestPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ClarificationRequestPrxHelper __h = new ClarificationRequestPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ClarificationRequestDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ClarificationRequestDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ClarificationRequestPrx v)
    {
        __os.writeProxy(v);
    }

    public static ClarificationRequestPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ClarificationRequestPrxHelper result = new ClarificationRequestPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
