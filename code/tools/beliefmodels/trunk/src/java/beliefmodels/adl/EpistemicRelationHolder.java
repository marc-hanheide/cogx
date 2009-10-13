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

public final class EpistemicRelationHolder
{
    public
    EpistemicRelationHolder()
    {
    }

    public
    EpistemicRelationHolder(EpistemicRelation value)
    {
        this.value = value;
    }

    public class Patcher implements IceInternal.Patcher
    {
        public void
        patch(Ice.Object v)
        {
            try
            {
                value = (EpistemicRelation)v;
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return "::beliefmodels::adl::EpistemicRelation";
        }
    }

    public Patcher
    getPatcher()
    {
        return new Patcher();
    }

    public EpistemicRelation value;
}
