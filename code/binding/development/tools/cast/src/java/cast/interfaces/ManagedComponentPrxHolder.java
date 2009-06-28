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

public final class ManagedComponentPrxHolder
{
    public
    ManagedComponentPrxHolder()
    {
    }

    public
    ManagedComponentPrxHolder(ManagedComponentPrx value)
    {
        this.value = value;
    }

    public ManagedComponentPrx value;
}
