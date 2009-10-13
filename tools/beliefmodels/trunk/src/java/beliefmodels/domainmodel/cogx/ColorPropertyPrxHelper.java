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

public final class ColorPropertyPrxHelper extends Ice.ObjectPrxHelperBase implements ColorPropertyPrx
{
    public static ColorPropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ColorPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ColorPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ColorProperty"))
                {
                    ColorPropertyPrxHelper __h = new ColorPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ColorPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ColorPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ColorPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ColorProperty", __ctx))
                {
                    ColorPropertyPrxHelper __h = new ColorPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ColorPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ColorPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ColorProperty"))
                {
                    ColorPropertyPrxHelper __h = new ColorPropertyPrxHelper();
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

    public static ColorPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ColorPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ColorProperty", __ctx))
                {
                    ColorPropertyPrxHelper __h = new ColorPropertyPrxHelper();
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

    public static ColorPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ColorPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ColorPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ColorPropertyPrxHelper __h = new ColorPropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ColorPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ColorPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ColorPropertyPrxHelper __h = new ColorPropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ColorPropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ColorPropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ColorPropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static ColorPropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ColorPropertyPrxHelper result = new ColorPropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
