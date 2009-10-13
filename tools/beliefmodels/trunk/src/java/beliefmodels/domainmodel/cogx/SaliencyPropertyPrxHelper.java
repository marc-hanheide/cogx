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

public final class SaliencyPropertyPrxHelper extends Ice.ObjectPrxHelperBase implements SaliencyPropertyPrx
{
    public static SaliencyPropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        SaliencyPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SaliencyPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::SaliencyProperty"))
                {
                    SaliencyPropertyPrxHelper __h = new SaliencyPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SaliencyPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        SaliencyPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SaliencyPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::SaliencyProperty", __ctx))
                {
                    SaliencyPropertyPrxHelper __h = new SaliencyPropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static SaliencyPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SaliencyPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::SaliencyProperty"))
                {
                    SaliencyPropertyPrxHelper __h = new SaliencyPropertyPrxHelper();
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

    public static SaliencyPropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        SaliencyPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::SaliencyProperty", __ctx))
                {
                    SaliencyPropertyPrxHelper __h = new SaliencyPropertyPrxHelper();
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

    public static SaliencyPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        SaliencyPropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (SaliencyPropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                SaliencyPropertyPrxHelper __h = new SaliencyPropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static SaliencyPropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        SaliencyPropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            SaliencyPropertyPrxHelper __h = new SaliencyPropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _SaliencyPropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _SaliencyPropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, SaliencyPropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static SaliencyPropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            SaliencyPropertyPrxHelper result = new SaliencyPropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
