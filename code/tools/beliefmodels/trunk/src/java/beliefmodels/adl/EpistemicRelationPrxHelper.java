// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.adl;

public final class EpistemicRelationPrxHelper extends Ice.ObjectPrxHelperBase implements EpistemicRelationPrx
{
    public static EpistemicRelationPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        EpistemicRelationPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicRelationPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::EpistemicRelation"))
                {
                    EpistemicRelationPrxHelper __h = new EpistemicRelationPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EpistemicRelationPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        EpistemicRelationPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicRelationPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA("::beliefmodels::adl::EpistemicRelation", __ctx))
                {
                    EpistemicRelationPrxHelper __h = new EpistemicRelationPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static EpistemicRelationPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EpistemicRelationPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::EpistemicRelation"))
                {
                    EpistemicRelationPrxHelper __h = new EpistemicRelationPrxHelper();
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

    public static EpistemicRelationPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        EpistemicRelationPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA("::beliefmodels::adl::EpistemicRelation", __ctx))
                {
                    EpistemicRelationPrxHelper __h = new EpistemicRelationPrxHelper();
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

    public static EpistemicRelationPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        EpistemicRelationPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (EpistemicRelationPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                EpistemicRelationPrxHelper __h = new EpistemicRelationPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static EpistemicRelationPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        EpistemicRelationPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            EpistemicRelationPrxHelper __h = new EpistemicRelationPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _EpistemicRelationDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _EpistemicRelationDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, EpistemicRelationPrx v)
    {
        __os.writeProxy(v);
    }

    public static EpistemicRelationPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            EpistemicRelationPrxHelper result = new EpistemicRelationPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
