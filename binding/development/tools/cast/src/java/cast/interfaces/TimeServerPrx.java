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

public interface TimeServerPrx extends Ice.ObjectPrx
{
    public cast.cdl.CASTTime getCASTTime();
    public cast.cdl.CASTTime getCASTTime(java.util.Map<String, String> __ctx);

    public cast.cdl.CASTTime fromTimeOfDayDouble(double todsecs);
    public cast.cdl.CASTTime fromTimeOfDayDouble(double todsecs, java.util.Map<String, String> __ctx);

    public cast.cdl.CASTTime fromTimeOfDay(long secs, long usecs);
    public cast.cdl.CASTTime fromTimeOfDay(long secs, long usecs, java.util.Map<String, String> __ctx);

    public void reset();
    public void reset(java.util.Map<String, String> __ctx);
}
