// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.epstatus;

public final class AgentsHelper
{
    public static void
    write(IceInternal.BasicStream __os, java.util.List<java.lang.String> __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.size());
            for(String __elem : __v)
            {
                __os.writeString(__elem);
            }
        }
    }

    public static java.util.List<java.lang.String>
    read(IceInternal.BasicStream __is)
    {
        java.util.List<java.lang.String> __v;
        __v = new java.util.LinkedList<String>();
        final int __len0 = __is.readSize();
        __is.startSeq(__len0, 1);
        for(int __i0 = 0; __i0 < __len0; __i0++)
        {
            String __elem;
            __elem = __is.readString();
            __v.add(__elem);
            __is.checkSeq();
            __is.endElement();
        }
        __is.endSeq(__len0);
        return __v;
    }
}
