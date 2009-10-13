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

public final class BeliefPrxHelper extends Ice.ObjectPrxHelperBase implements BeliefPrx
{
    public static BeliefPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        BeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Belief"))
                {
                    BeliefPrxHelper __h = new BeliefPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BeliefPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        BeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::Belief", __ctx))
                {
                    BeliefPrxHelper __h = new BeliefPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BeliefPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Belief"))
                {
                    BeliefPrxHelper __h = new BeliefPrxHelper();
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

    public static BeliefPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        BeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::Belief", __ctx))
                {
                    BeliefPrxHelper __h = new BeliefPrxHelper();
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

    public static BeliefPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        BeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                BeliefPrxHelper __h = new BeliefPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static BeliefPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            BeliefPrxHelper __h = new BeliefPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _BeliefDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _BeliefDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, BeliefPrx v)
    {
        __os.writeProxy(v);
    }

    public static BeliefPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            BeliefPrxHelper result = new BeliefPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
