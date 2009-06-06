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

public final class ReactionPrxHelper extends Ice.ObjectPrxHelperBase implements ReactionPrx
{
    public static ReactionPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ReactionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ReactionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::Reaction"))
                {
                    ReactionPrxHelper __h = new ReactionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ReactionPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ReactionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ReactionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::comedyarch::autogen::Reaction", __ctx))
                {
                    ReactionPrxHelper __h = new ReactionPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ReactionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ReactionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::Reaction"))
                {
                    ReactionPrxHelper __h = new ReactionPrxHelper();
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

    public static ReactionPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ReactionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::comedyarch::autogen::Reaction", __ctx))
                {
                    ReactionPrxHelper __h = new ReactionPrxHelper();
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

    public static ReactionPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ReactionPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ReactionPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ReactionPrxHelper __h = new ReactionPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ReactionPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ReactionPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ReactionPrxHelper __h = new ReactionPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ReactionDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ReactionDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ReactionPrx v)
    {
        __os.writeProxy(v);
    }

    public static ReactionPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ReactionPrxHelper result = new ReactionPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
