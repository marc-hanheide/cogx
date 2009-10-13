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

public class Task extends EpistemicObject
{
    public Task()
    {
        super();
    }

    public Task(String id, SpatioTemporalFrame sigma, Agent[] ags, GoalFormula goal)
    {
        super(id);
        this.sigma = sigma;
        this.ags = ags;
        this.goal = goal;
    }

    private static class __F implements Ice.ObjectFactory
    {
        public Ice.Object
        create(String type)
        {
            assert(type.equals(ice_staticId()));
            return new Task();
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
        "::beliefmodels::adl::EpistemicObject",
        "::beliefmodels::adl::Task"
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
        return __ids[2];
    }

    public String
    ice_id(Ice.Current __current)
    {
        return __ids[2];
    }

    public static String
    ice_staticId()
    {
        return __ids[2];
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeTypeId(ice_staticId());
        __os.startWriteSlice();
        __os.writeObject(sigma);
        AgentsHelper.write(__os, ags);
        __os.writeObject(goal);
        __os.endWriteSlice();
        super.__write(__os);
    }

    private class Patcher implements IceInternal.Patcher
    {
        Patcher(int member)
        {
            __member = member;
        }

        public void
        patch(Ice.Object v)
        {
            try
            {
                switch(__member)
                {
                case 0:
                    __typeId = "::beliefmodels::adl::SpatioTemporalFrame";
                    sigma = (SpatioTemporalFrame)v;
                    break;
                case 1:
                    __typeId = "::beliefmodels::adl::GoalFormula";
                    goal = (GoalFormula)v;
                    break;
                }
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return __typeId;
        }

        private int __member;
        private String __typeId;
    }

    public void
    __read(IceInternal.BasicStream __is, boolean __rid)
    {
        if(__rid)
        {
            __is.readTypeId();
        }
        __is.startReadSlice();
        __is.readObject(new Patcher(0));
        ags = AgentsHelper.read(__is);
        __is.readObject(new Patcher(1));
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type beliefmodels::adl::Task was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type beliefmodels::adl::Task was not generated with stream support";
        throw ex;
    }

    public SpatioTemporalFrame sigma;

    public Agent[] ags;

    public GoalFormula goal;
}
