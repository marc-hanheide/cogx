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

public final class JokePrxHelper extends Ice.ObjectPrxHelperBase implements JokePrx
{
    public static JokePrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        JokePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (JokePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::Joke"))
                {
                    JokePrxHelper __h = new JokePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static JokePrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        JokePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (JokePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::Joke", __ctx))
                {
                    JokePrxHelper __h = new JokePrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static JokePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        JokePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::Joke"))
                {
                    JokePrxHelper __h = new JokePrxHelper();
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

    public static JokePrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        JokePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::Joke", __ctx))
                {
                    JokePrxHelper __h = new JokePrxHelper();
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

    public static JokePrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        JokePrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (JokePrx)__obj;
            }
            catch(ClassCastException ex)
            {
                JokePrxHelper __h = new JokePrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static JokePrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        JokePrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            JokePrxHelper __h = new JokePrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _JokeDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _JokeDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, JokePrx v)
    {
        __os.writeProxy(v);
    }

    public static JokePrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            JokePrxHelper result = new JokePrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
