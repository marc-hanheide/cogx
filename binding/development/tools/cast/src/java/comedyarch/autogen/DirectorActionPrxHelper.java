// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package comedyarch.autogen;

public final class DirectorActionPrxHelper extends Ice.ObjectPrxHelperBase implements DirectorActionPrx
{
    public static DirectorActionPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        DirectorActionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DirectorActionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::DirectorAction"))
                {
                    DirectorActionPrxHelper __h = new DirectorActionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static DirectorActionPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        DirectorActionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DirectorActionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::DirectorAction", __ctx))
                {
                    DirectorActionPrxHelper __h = new DirectorActionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static DirectorActionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        DirectorActionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::DirectorAction"))
                {
                    DirectorActionPrxHelper __h = new DirectorActionPrxHelper();
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

    public static DirectorActionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        DirectorActionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::DirectorAction", __ctx))
                {
                    DirectorActionPrxHelper __h = new DirectorActionPrxHelper();
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

    public static DirectorActionPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        DirectorActionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DirectorActionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                DirectorActionPrxHelper __h = new DirectorActionPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static DirectorActionPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        DirectorActionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            DirectorActionPrxHelper __h = new DirectorActionPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _DirectorActionDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _DirectorActionDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, DirectorActionPrx v)
    {
        __os.writeProxy(v);
    }

    public static DirectorActionPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            DirectorActionPrxHelper result = new DirectorActionPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
