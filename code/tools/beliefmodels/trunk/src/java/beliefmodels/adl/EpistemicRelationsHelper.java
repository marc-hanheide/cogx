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

public final class EpistemicRelationsHelper
{
    public static void
    write(IceInternal.BasicStream __os, EpistemicRelation[] __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.length);
            for(int __i0 = 0; __i0 < __v.length; __i0++)
            {
                __os.writeObject(__v[__i0]);
            }
        }
    }

    public static EpistemicRelation[]
    read(IceInternal.BasicStream __is)
    {
        EpistemicRelation[] __v;
        final int __len0 = __is.readSize();
        __is.startSeq(__len0, 4);
        final String __type0 = EpistemicRelation.ice_staticId();
        __v = new EpistemicRelation[__len0];
        for(int __i0 = 0; __i0 < __len0; __i0++)
        {
            __is.readObject(new IceInternal.SequencePatcher(__v, EpistemicRelation.class, __type0, __i0));
            __is.checkSeq();
            __is.endElement();
        }
        __is.endSeq(__len0);
        return __v;
    }
}
