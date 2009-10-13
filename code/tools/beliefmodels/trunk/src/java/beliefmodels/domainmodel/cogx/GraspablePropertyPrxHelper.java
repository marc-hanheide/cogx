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

public final class GraspablePropertyPrxHelper extends Ice.ObjectPrxHelperBase implements GraspablePropertyPrx
{
    public static GraspablePropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        GraspablePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GraspablePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::GraspableProperty"))
                {
                    GraspablePropertyPrxHelper __h = new GraspablePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GraspablePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        GraspablePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GraspablePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::GraspableProperty", __ctx))
                {
                    GraspablePropertyPrxHelper __h = new GraspablePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static GraspablePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GraspablePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::GraspableProperty"))
                {
                    GraspablePropertyPrxHelper __h = new GraspablePropertyPrxHelper();
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

    public static GraspablePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        GraspablePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::GraspableProperty", __ctx))
                {
                    GraspablePropertyPrxHelper __h = new GraspablePropertyPrxHelper();
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

    public static GraspablePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        GraspablePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (GraspablePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                GraspablePropertyPrxHelper __h = new GraspablePropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static GraspablePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        GraspablePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            GraspablePropertyPrxHelper __h = new GraspablePropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _GraspablePropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _GraspablePropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, GraspablePropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static GraspablePropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            GraspablePropertyPrxHelper result = new GraspablePropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
