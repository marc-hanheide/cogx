// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package beliefmodels.clarification;

public final class ClarificationRequestHolder
{
    public
    ClarificationRequestHolder()
    {
    }

    public
    ClarificationRequestHolder(ClarificationRequest value)
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
                value = (ClarificationRequest)v;
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return "::beliefmodels::clarification::ClarificationRequest";
        }
    }

    public Patcher
    getPatcher()
    {
        return new Patcher();
    }

    public ClarificationRequest value;
}
