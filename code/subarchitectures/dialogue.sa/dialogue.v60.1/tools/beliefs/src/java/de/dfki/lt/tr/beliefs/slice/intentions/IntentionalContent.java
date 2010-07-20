// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.intentions;

public class IntentionalContent extends Ice.ObjectImpl
{
    public IntentionalContent()
    {
    }

    public IntentionalContent(java.util.List<java.lang.String> agents, de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula preconditions, de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula postconditions, float probValue)
    {
        this.agents = agents;
        this.preconditions = preconditions;
        this.postconditions = postconditions;
        this.probValue = probValue;
    }

    private static class __F implements Ice.ObjectFactory
    {
        public Ice.Object
        create(String type)
        {
            assert(type.equals(ice_staticId()));
            return new IntentionalContent();
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
        "::de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent"
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
        de.dfki.lt.tr.beliefs.slice.epstatus.AgentsHelper.write(__os, agents);
        __os.writeObject(preconditions);
        __os.writeObject(postconditions);
        __os.writeFloat(probValue);
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
                    __typeId = "::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula";
                    preconditions = (de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula)v;
                    break;
                case 1:
                    __typeId = "::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula";
                    postconditions = (de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula)v;
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
        agents = de.dfki.lt.tr.beliefs.slice.epstatus.AgentsHelper.read(__is);
        __is.readObject(new Patcher(0));
        __is.readObject(new Patcher(1));
        probValue = __is.readFloat();
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent was not generated with stream support";
        throw ex;
    }

    public java.util.List<java.lang.String> agents;

    public de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula preconditions;

    public de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula postconditions;

    public float probValue;
}
