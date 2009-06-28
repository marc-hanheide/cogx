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

public interface _WorkingMemoryOperationsNC extends _CASTComponentOperationsNC
{
    boolean exists(String id, String subarch)
        throws cast.UnknownSubarchitectureException;

    int getVersionNumber(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    cast.cdl.WorkingMemoryPermissions getPermissions(String id, String subarch)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    boolean tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void unlockEntry(String id, String subarch, String component)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void setWorkingMemory(WorkingMemoryPrx wm, String subarch);

    void addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException;

    void overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void deleteFromWorkingMemory(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    cast.cdl.WorkingMemoryEntry getWorkingMemoryEntry(String id, String subarch, String component)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries)
        throws cast.UnknownSubarchitectureException;

    void registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter);

    void removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter);

    void registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch);

    void removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter);

    void addReader(WorkingMemoryReaderComponentPrx reader);

    void receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc);
}
