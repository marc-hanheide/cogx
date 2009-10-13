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

public final class MutualAgentStatusPrxHelper extends Ice.ObjectPrxHelperBase implements MutualAgentStatusPrx
{
    public static MutualAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        MutualAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (MutualAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::MutualAgentStatus"))
                {
                    MutualAgentStatusPrxHelper __h = new MutualAgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static MutualAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        MutualAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (MutualAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::MutualAgentStatus", __ctx))
                {
                    MutualAgentStatusPrxHelper __h = new MutualAgentStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static MutualAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        MutualAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::MutualAgentStatus"))
                {
                    MutualAgentStatusPrxHelper __h = new MutualAgentStatusPrxHelper();
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

    public static MutualAgentStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        MutualAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::MutualAgentStatus", __ctx))
                {
                    MutualAgentStatusPrxHelper __h = new MutualAgentStatusPrxHelper();
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

    public static MutualAgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        MutualAgentStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (MutualAgentStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                MutualAgentStatusPrxHelper __h = new MutualAgentStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static MutualAgentStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        MutualAgentStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            MutualAgentStatusPrxHelper __h = new MutualAgentStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _MutualAgentStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _MutualAgentStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, MutualAgentStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static MutualAgentStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            MutualAgentStatusPrxHelper result = new MutualAgentStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
