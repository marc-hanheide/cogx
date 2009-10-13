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

public final class ShapePropertyPrxHelper extends Ice.ObjectPrxHelperBase implements ShapePropertyPrx
{
    public static ShapePropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ShapePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ShapePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ShapeProperty"))
                {
                    ShapePropertyPrxHelper __h = new ShapePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ShapePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ShapePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ShapePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ShapeProperty", __ctx))
                {
                    ShapePropertyPrxHelper __h = new ShapePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ShapePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ShapePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ShapeProperty"))
                {
                    ShapePropertyPrxHelper __h = new ShapePropertyPrxHelper();
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

    public static ShapePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ShapePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ShapeProperty", __ctx))
                {
                    ShapePropertyPrxHelper __h = new ShapePropertyPrxHelper();
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

    public static ShapePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ShapePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ShapePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ShapePropertyPrxHelper __h = new ShapePropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ShapePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ShapePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ShapePropertyPrxHelper __h = new ShapePropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ShapePropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ShapePropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ShapePropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static ShapePropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ShapePropertyPrxHelper result = new ShapePropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
