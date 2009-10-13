// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.adl;

public final class PrivateAgentStatusPrxHelper extends Ice.ObjectPrxHelperBase implements PrivateAgentStatusPrx
{
    public static PrivateAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        PrivateAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PrivateAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::PrivateAgentStatus"))
                {
                    PrivateAgentStatusPrxHelper __h = new PrivateAgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static PrivateAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        PrivateAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PrivateAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::PrivateAgentStatus", __ctx))
                {
                    PrivateAgentStatusPrxHelper __h = new PrivateAgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static PrivateAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        PrivateAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::PrivateAgentStatus"))
                {
                    PrivateAgentStatusPrxHelper __h = new PrivateAgentStatusPrxHelper();
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

    public static PrivateAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        PrivateAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::PrivateAgentStatus", __ctx))
                {
                    PrivateAgentStatusPrxHelper __h = new PrivateAgentStatusPrxHelper();
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

    public static PrivateAgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        PrivateAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (PrivateAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                PrivateAgentStatusPrxHelper __h = new PrivateAgentStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static PrivateAgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        PrivateAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            PrivateAgentStatusPrxHelper __h = new PrivateAgentStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _PrivateAgentStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _PrivateAgentStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, PrivateAgentStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static PrivateAgentStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            PrivateAgentStatusPrxHelper result = new PrivateAgentStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
