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

public final class AgentPrxHelper extends Ice.ObjectPrxHelperBase implements AgentPrx
{
    public static AgentPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        AgentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AgentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Agent"))
                {
                    AgentPrxHelper __h = new AgentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AgentPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        AgentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AgentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Agent", __ctx))
                {
                    AgentPrxHelper __h = new AgentPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static AgentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AgentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Agent"))
                {
                    AgentPrxHelper __h = new AgentPrxHelper();
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

    public static AgentPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        AgentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Agent", __ctx))
                {
                    AgentPrxHelper __h = new AgentPrxHelper();
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

    public static AgentPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        AgentPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (AgentPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                AgentPrxHelper __h = new AgentPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static AgentPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        AgentPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            AgentPrxHelper __h = new AgentPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _AgentDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _AgentDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, AgentPrx v)
    {
        __os.writeProxy(v);
    }

    public static AgentPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            AgentPrxHelper result = new AgentPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
