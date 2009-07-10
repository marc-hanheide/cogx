// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.interfaces;

public interface _TimeServerOperations
{
    cast.cdl.CASTTime getCASTTime(Ice.Current __current);

    cast.cdl.CASTTime fromTimeOfDayDouble(double todsecs, Ice.Current __current);

    cast.cdl.CASTTime fromTimeOfDay(long secs, long usecs, Ice.Current __current);

    void reset(Ice.Current __current);
}
