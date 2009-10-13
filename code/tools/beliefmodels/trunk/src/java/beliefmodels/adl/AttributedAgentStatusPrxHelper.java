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

public final class AttributedAgentStatusPrxHelper extends Ice.ObjectPrxHelperBase implements AttributedAgentStatusPrx
{
    public static AttributedAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        AttributedAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AttributedAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::AttributedAgentStatus"))
                {
                    AttributedAgentStatusPrxHelper __h = new AttributedAgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AttributedAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        AttributedAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AttributedAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::AttributedAgentStatus", __ctx))
                {
                    AttributedAgentStatusPrxHelper __h = new AttributedAgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AttributedAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AttributedAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::AttributedAgentStatus"))
                {
                    AttributedAgentStatusPrxHelper __h = new AttributedAgentStatusPrxHelper();
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

    public static AttributedAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        AttributedAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::AttributedAgentStatus", __ctx))
                {
                    AttributedAgentStatusPrxHelper __h = new AttributedAgentStatusPrxHelper();
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

    public static AttributedAgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        AttributedAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AttributedAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                AttributedAgentStatusPrxHelper __h = new AttributedAgentStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static AttributedAgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AttributedAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            AttributedAgentStatusPrxHelper __h = new AttributedAgentStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _AttributedAgentStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _AttributedAgentStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, AttributedAgentStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static AttributedAgentStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            AttributedAgentStatusPrxHelper result = new AttributedAgentStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
