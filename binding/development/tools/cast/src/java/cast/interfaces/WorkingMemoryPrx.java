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

public interface WorkingMemoryPrx extends CASTComponentPrx
{
    public boolean exists(String id, String subarch)
        throws cast.UnknownSubarchitectureException;
    public boolean exists(String id, String subarch, java.util.Map<String, String> __ctx)
        throws cast.UnknownSubarchitectureException;

    public int getVersionNumber(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public int getVersionNumber(String id, String subarch, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public cast.cdl.WorkingMemoryPermissions getPermissions(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public cast.cdl.WorkingMemoryPermissions getPermissions(String id, String subarch, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public void lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public void lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public boolean tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public boolean tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public void unlockEntry(String id, String subarch, String component)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public void unlockEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public void setWorkingMemory(WorkingMemoryPrx wm, String subarch);
    public void setWorkingMemory(WorkingMemoryPrx wm, String subarch, java.util.Map<String, String> __ctx);

    public void addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException;
    public void addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException;

    public void overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public void overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public void deleteFromWorkingMemory(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public void deleteFromWorkingMemory(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public cast.cdl.WorkingMemoryEntry getWorkingMemoryEntry(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;
    public cast.cdl.WorkingMemoryEntry getWorkingMemoryEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    public void getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries)
        throws cast.UnknownSubarchitectureException;
    public void getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, java.util.Map<String, String> __ctx)
        throws cast.UnknownSubarchitectureException;

    public void registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter);
    public void registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx);

    public void removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter);
    public void removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx);

    public void registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch);
    public void registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, java.util.Map<String, String> __ctx);

    public void removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter);
    public void removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx);

    public void addReader(WorkingMemoryReaderComponentPrx reader);
    public void addReader(WorkingMemoryReaderComponentPrx reader, java.util.Map<String, String> __ctx);

    public void receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc);
    public void receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, java.util.Map<String, String> __ctx);
}
