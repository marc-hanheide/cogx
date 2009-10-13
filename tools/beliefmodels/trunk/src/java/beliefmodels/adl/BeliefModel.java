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

public class BeliefModel extends Ice.ObjectImpl
{
    public BeliefModel()
    {
    }

    public BeliefModel(String id, Agent[] a, SpatioTemporalModel s, String[] k, String[] t, String[] f)
    {
        this.id = id;
        this.a = a;
        this.s = s;
        this.k = k;
        this.t = t;
        this.f = f;
    }

    private static class __F implements Ice.ObjectFactory
    {
        public Ice.Object
        create(String type)
        {
            assert(type.equals(ice_staticId()));
            return new BeliefModel();
        }

        public void
        destroy()
        {
        }
    }
    private static Ice.ObjectFactory _factory = new __F();

    public static Ice.ObjectFactory
    ice_factory()
    {
        return _factory;
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::beliefmodels::adl::BeliefModel"
    };

    public boolean
    ice_isA(String s)
    {
        return java.util.Arrays.binarySearch(__ids, s) >= 0;
    }

    public boolean
    ice_isA(String s, Ice.Current __current)
    {
        return java.util.Arrays.binarySearch(__ids, s) >= 0;
    }

    public String[]
    ice_ids()
    {
        return __ids;
    }

    public String[]
    ice_ids(Ice.Current __current)
    {
        return __ids;
    }

    public String
    ice_id()
    {
        return __ids[1];
    }

    public String
    ice_id(Ice.Current __current)
    {
        return __ids[1];
    }

    public static String
    ice_staticId()
    {
        return __ids[1];
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeTypeId(ice_staticId());
        __os.startWriteSlice();
        __os.writeString(id);
        AgentsHelper.write(__os, a);
        __os.writeObject(s);
        BeliefPointersHelper.write(__os, k);
        TaskPointersHelper.write(__os, t);
        ForegroundHelper.write(__os, f);
        __os.endWriteSlice();
        super.__write(__os);
    }

    private class Patcher implements IceInternal.Patcher
    {
        public void
        patch(Ice.Object v)
        {
            try
            {
                s = (SpatioTemporalModel)v;
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return "::beliefmodels::adl::SpatioTemporalModel";
        }
    }

    public void
    __read(IceInternal.BasicStream __is, boolean __rid)
    {
        if(__rid)
        {
            __is.readTypeId();
        }
        __is.startReadSlice();
        id = __is.readString();
        a = AgentsHelper.read(__is);
        __is.readObject(new Patcher());
        k = BeliefPointersHelper.read(__is);
        t = TaskPointersHelper.read(__is);
        f = ForegroundHelper.read(__is);
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type beliefmodels::adl::BeliefModel was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type beliefmodels::adl::BeliefModel was not generated with stream support";
        throw ex;
    }

    public String id;

    public Agent[] a;

    public SpatioTemporalModel s;

    public String[] k;

    public String[] t;

    public String[] f;
}
