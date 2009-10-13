// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.domainmodel.cogx;

public final class SaliencyAssignmentHelper
{
    public static void
    write(IceInternal.BasicStream __os, java.util.Map<java.lang.String, java.lang.Double> __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.size());
            for(java.util.Map.Entry<java.lang.String, java.lang.Double> __e : __v.entrySet())
            {
                __os.writeString(__e.getKey());
                __os.writeDouble(__e.getValue());
            }
        }
    }

    public static java.util.Map<java.lang.String, java.lang.Double>
    read(IceInternal.BasicStream __is)
    {
        java.util.Map<java.lang.String, java.lang.Double> __v;
        __v = new java.util.HashMap<java.lang.String, java.lang.Double>();
        int __sz0 = __is.readSize();
        for(int __i0 = 0; __i0 < __sz0; __i0++)
        {
            String __key;
            __key = __is.readString();
            double __value;
            __value = __is.readDouble();
            __v.put(__key, __value);
        }
        return __v;
    }
}
