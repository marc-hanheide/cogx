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

public final class AgentStatusPrxHelper extends Ice.ObjectPrxHelperBase implements AgentStatusPrx
{
    public static AgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        AgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::AgentStatus"))
                {
                    AgentStatusPrxHelper __h = new AgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        AgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::AgentStatus", __ctx))
                {
                    AgentStatusPrxHelper __h = new AgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::AgentStatus"))
                {
                    AgentStatusPrxHelper __h = new AgentStatusPrxHelper();
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

    public static AgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        AgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::AgentStatus", __ctx))
                {
                    AgentStatusPrxHelper __h = new AgentStatusPrxHelper();
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

    public static AgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        AgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                AgentStatusPrxHelper __h = new AgentStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static AgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            AgentStatusPrxHelper __h = new AgentStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _AgentStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _AgentStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, AgentStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static AgentStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            AgentStatusPrxHelper result = new AgentStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
