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

public final class ObjectTypePropertyPrxHelper extends Ice.ObjectPrxHelperBase implements ObjectTypePropertyPrx
{
    public static ObjectTypePropertyPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ObjectTypePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ObjectTypePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ObjectTypeProperty"))
                {
                    ObjectTypePropertyPrxHelper __h = new ObjectTypePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ObjectTypePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ObjectTypePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ObjectTypePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::domainmodel::cogx::ObjectTypeProperty", __ctx))
                {
                    ObjectTypePropertyPrxHelper __h = new ObjectTypePropertyPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ObjectTypePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ObjectTypePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ObjectTypeProperty"))
                {
                    ObjectTypePropertyPrxHelper __h = new ObjectTypePropertyPrxHelper();
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

    public static ObjectTypePropertyPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ObjectTypePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::domainmodel::cogx::ObjectTypeProperty", __ctx))
                {
                    ObjectTypePropertyPrxHelper __h = new ObjectTypePropertyPrxHelper();
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

    public static ObjectTypePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ObjectTypePropertyPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ObjectTypePropertyPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ObjectTypePropertyPrxHelper __h = new ObjectTypePropertyPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ObjectTypePropertyPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ObjectTypePropertyPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ObjectTypePropertyPrxHelper __h = new ObjectTypePropertyPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ObjectTypePropertyDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ObjectTypePropertyDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ObjectTypePropertyPrx v)
    {
        __os.writeProxy(v);
    }

    public static ObjectTypePropertyPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ObjectTypePropertyPrxHelper result = new ObjectTypePropertyPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
