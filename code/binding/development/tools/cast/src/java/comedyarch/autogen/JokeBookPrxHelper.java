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

public final class JokeBookPrxHelper extends Ice.ObjectPrxHelperBase implements JokeBookPrx
{
    public static JokeBookPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        JokeBookPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (JokeBookPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::JokeBook"))
                {
                    JokeBookPrxHelper __h = new JokeBookPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static JokeBookPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        JokeBookPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (JokeBookPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::JokeBook", __ctx))
                {
                    JokeBookPrxHelper __h = new JokeBookPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static JokeBookPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        JokeBookPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::JokeBook"))
                {
                    JokeBookPrxHelper __h = new JokeBookPrxHelper();
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

    public static JokeBookPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        JokeBookPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::JokeBook", __ctx))
                {
                    JokeBookPrxHelper __h = new JokeBookPrxHelper();
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

    public static JokeBookPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        JokeBookPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (JokeBookPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                JokeBookPrxHelper __h = new JokeBookPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static JokeBookPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        JokeBookPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            JokeBookPrxHelper __h = new JokeBookPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _JokeBookDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _JokeBookDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, JokeBookPrx v)
    {
        __os.writeProxy(v);
    }

    public static JokeBookPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            JokeBookPrxHelper result = new JokeBookPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
