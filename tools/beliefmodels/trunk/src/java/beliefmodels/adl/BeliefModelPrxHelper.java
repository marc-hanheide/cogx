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

public final class BeliefModelPrxHelper extends Ice.ObjectPrxHelperBase implements BeliefModelPrx
{
    public static BeliefModelPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        BeliefModelPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BeliefModelPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::BeliefModel"))
                {
                    BeliefModelPrxHelper __h = new BeliefModelPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BeliefModelPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        BeliefModelPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BeliefModelPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::BeliefModel", __ctx))
                {
                    BeliefModelPrxHelper __h = new BeliefModelPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BeliefModelPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BeliefModelPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::BeliefModel"))
                {
                    BeliefModelPrxHelper __h = new BeliefModelPrxHelper();
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

    public static BeliefModelPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        BeliefModelPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::BeliefModel", __ctx))
                {
                    BeliefModelPrxHelper __h = new BeliefModelPrxHelper();
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

    public static BeliefModelPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        BeliefModelPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BeliefModelPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                BeliefModelPrxHelper __h = new BeliefModelPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static BeliefModelPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BeliefModelPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            BeliefModelPrxHelper __h = new BeliefModelPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _BeliefModelDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _BeliefModelDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, BeliefModelPrx v)
    {
        __os.writeProxy(v);
    }

    public static BeliefModelPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            BeliefModelPrxHelper result = new BeliefModelPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
