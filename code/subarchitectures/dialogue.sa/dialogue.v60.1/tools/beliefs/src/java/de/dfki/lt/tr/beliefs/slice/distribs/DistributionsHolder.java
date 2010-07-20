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

public final class DistributionsHolder
{
    public
    DistributionsHolder()
    {
    }

    public
    DistributionsHolder(java.util.Map<java.lang.String, ProbDistribution> value)
    {
        this.value = value;
    }

    public java.util.Map<java.lang.String, ProbDistribution> value;
}
