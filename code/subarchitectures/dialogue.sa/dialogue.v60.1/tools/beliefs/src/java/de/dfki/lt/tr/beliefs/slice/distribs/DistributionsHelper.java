// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.distribs;

public final class DistributionsHelper
{
    public static void
    write(IceInternal.BasicStream __os, java.util.Map<java.lang.String, ProbDistribution> __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.size());
            for(java.util.Map.Entry<java.lang.String, ProbDistribution> __e : __v.entrySet())
            {
                __os.writeString(__e.getKey());
                __os.writeObject(__e.getValue());
            }
        }
    }

    public static java.util.Map<java.lang.String, ProbDistribution>
    read(IceInternal.BasicStream __is)
    {
        java.util.Map<java.lang.String, ProbDistribution> __v;
        __v = new java.util.HashMap<java.lang.String, de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution>();
        int __sz0 = __is.readSize();
        for(int __i0 = 0; __i0 < __sz0; __i0++)
        {
            String __key;
            __key = __is.readString();
            __is.readObject(new IceInternal.DictionaryPatcher<java.lang.String, ProbDistribution>(__v, ProbDistribution.class, "::de::dfki::lt::tr::beliefs::slice::distribs::ProbDistribution", __key));
        }
        return __v;
    }
}
