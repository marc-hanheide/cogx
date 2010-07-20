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

public final class FormulaProbPairsHelper
{
    public static void
    write(IceInternal.BasicStream __os, java.util.List<FormulaProbPair> __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.size());
            for(FormulaProbPair __elem : __v)
            {
                __elem.__write(__os);
            }
        }
    }

    public static java.util.List<FormulaProbPair>
    read(IceInternal.BasicStream __is)
    {
        java.util.List<FormulaProbPair> __v;
        __v = new java.util.LinkedList<FormulaProbPair>();
        final int __len0 = __is.readSize();
        __is.startSeq(__len0, 8);
        for(int __i0 = 0; __i0 < __len0; __i0++)
        {
            FormulaProbPair __elem;
            __elem = new FormulaProbPair();
            __elem.__read(__is);
            __v.add(__elem);
            __is.checkSeq();
            __is.endElement();
        }
        __is.endSeq(__len0);
        return __v;
    }
}
