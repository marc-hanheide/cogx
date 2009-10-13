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

public final class GroundedBeliefPrxHelper extends Ice.ObjectPrxHelperBase implements GroundedBeliefPrx
{
    public static GroundedBeliefPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        GroundedBeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GroundedBeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::GroundedBelief"))
                {
                    GroundedBeliefPrxHelper __h = new GroundedBeliefPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GroundedBeliefPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        GroundedBeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GroundedBeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::GroundedBelief", __ctx))
                {
                    GroundedBeliefPrxHelper __h = new GroundedBeliefPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GroundedBeliefPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GroundedBeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::GroundedBelief"))
                {
                    GroundedBeliefPrxHelper __h = new GroundedBeliefPrxHelper();
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

    public static GroundedBeliefPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        GroundedBeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::GroundedBelief", __ctx))
                {
                    GroundedBeliefPrxHelper __h = new GroundedBeliefPrxHelper();
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

    public static GroundedBeliefPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        GroundedBeliefPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GroundedBeliefPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                GroundedBeliefPrxHelper __h = new GroundedBeliefPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static GroundedBeliefPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GroundedBeliefPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            GroundedBeliefPrxHelper __h = new GroundedBeliefPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _GroundedBeliefDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _GroundedBeliefDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, GroundedBeliefPrx v)
    {
        __os.writeProxy(v);
    }

    public static GroundedBeliefPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            GroundedBeliefPrxHelper result = new GroundedBeliefPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
