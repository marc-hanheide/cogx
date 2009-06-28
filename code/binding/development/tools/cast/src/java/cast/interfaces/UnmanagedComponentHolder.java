// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0

package cast.interfaces;

public final class UnmanagedComponentHolder
{
    public
    UnmanagedComponentHolder()
    {
    }

    public
    UnmanagedComponentHolder(UnmanagedComponent value)
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
                value = (UnmanagedComponent)v;
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return "::cast::interfaces::UnmanagedComponent";
        }
    }

    public Patcher
    getPatcher()
    {
        return new Patcher();
    }

    public UnmanagedComponent value;
}
