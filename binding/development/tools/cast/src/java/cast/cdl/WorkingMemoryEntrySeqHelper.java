// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.cdl;

public final class WorkingMemoryEntrySeqHelper
{
    public static void
    write(IceInternal.BasicStream __os, java.util.List<WorkingMemoryEntry> __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.size());
            for(WorkingMemoryEntry __elem : __v)
            {
                __os.writeObject(__elem);
            }
        }
    }

    public static java.util.List<WorkingMemoryEntry>
    read(IceInternal.BasicStream __is)
    {
        java.util.List<WorkingMemoryEntry> __v;
        __v = new java.util.LinkedList<WorkingMemoryEntry>();
        final int __len0 = __is.readSize();
        __is.startSeq(__len0, 4);
        final String __type0 = WorkingMemoryEntry.ice_staticId();
        for(int __i0 = 0; __i0 < __len0; __i0++)
        {
            __v.add(null);
            __is.readObject(new IceInternal.ListPatcher<WorkingMemoryEntry>(__v, WorkingMemoryEntry.class, __type0, __i0));
            __is.checkSeq();
            __is.endElement();
        }
        __is.endSeq(__len0);
        return __v;
    }
}
