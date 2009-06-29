// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.examples.autogen;

public final class WordServerHolder
{
    public
    WordServerHolder()
    {
    }

    public
    WordServerHolder(WordServer value)
    {
        this.value = value;
    }

    public class Patcher implements IceInternal.Patcher, Ice.ReadObjectCallback
    {
        public void
        patch(Ice.Object v)
        {
            try
            {
                value = (WordServer)v;
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return "::cast::examples::autogen::WordServer";
        }

        public void
        invoke(Ice.Object v)
        {
            patch(v);
        }
    }

    public Patcher
    getPatcher()
    {
        return new Patcher();
    }

    public WordServer value;
}
