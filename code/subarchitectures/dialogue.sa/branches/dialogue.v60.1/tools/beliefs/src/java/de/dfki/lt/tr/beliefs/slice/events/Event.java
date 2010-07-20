// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.events;

public class Event extends de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject
{
    public Event()
    {
        super();
    }

    public Event(de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame frame, de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus estatus, String id, de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution content)
    {
        super(frame, estatus);
        this.id = id;
        this.content = content;
    }

    private static class __F implements Ice.ObjectFactory
    {
        public Ice.Object
        create(String type)
        {
            assert(type.equals(ice_staticId()));
            return new Event();
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
        "::de::dfki::lt::tr::beliefs::slice::epobject::EpistemicObject",
        "::de::dfki::lt::tr::beliefs::slice::events::Event"
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
        __os.writeString(id);
        __os.writeObject(content);
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
                    __typeId = "::de::dfki::lt::tr::beliefs::slice::framing::AbstractFrame";
                    frame = (de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame)v;
                    break;
                case 1:
                    __typeId = "::de::dfki::lt::tr::beliefs::slice::epstatus::EpistemicStatus";
                    estatus = (de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus)v;
                    break;
                case 2:
                    __typeId = "::de::dfki::lt::tr::beliefs::slice::distribs::ProbDistribution";
                    content = (de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution)v;
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
        id = __is.readString();
        __is.readObject(new Patcher(2));
        __is.endReadSlice();
        super.__read(__is, true);
    }

    public void
    __write(Ice.OutputStream __outS)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type de::dfki::lt::tr::beliefs::slice::events::Event was not generated with stream support";
        throw ex;
    }

    public void
    __read(Ice.InputStream __inS, boolean __rid)
    {
        Ice.MarshalException ex = new Ice.MarshalException();
        ex.reason = "type de::dfki::lt::tr::beliefs::slice::events::Event was not generated with stream support";
        throw ex;
    }

    public String id;

    public de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution content;
}
