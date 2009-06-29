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

public interface _WorkingMemoryOperations extends _CASTComponentOperations
{
    boolean exists(String id, String subarch, Ice.Current __current)
        throws cast.UnknownSubarchitectureException;

    int getVersionNumber(String id, String subarch, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    cast.cdl.WorkingMemoryPermissions getPermissions(String id, String subarch, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    boolean tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void unlockEntry(String id, String subarch, String component, Ice.Current __current)
        throws cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void setWorkingMemory(WorkingMemoryPrx wm, String subarch, Ice.Current __current);

    void addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, Ice.Current __current)
        throws cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException;

    void overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void deleteFromWorkingMemory(String id, String subarch, String component, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    cast.cdl.WorkingMemoryEntry getWorkingMemoryEntry(String id, String subarch, String component, Ice.Current __current)
        throws cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, Ice.Current __current)
        throws cast.UnknownSubarchitectureException;

    void registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, Ice.Current __current);

    void removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, Ice.Current __current);

    void registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, Ice.Current __current);

    void removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, Ice.Current __current);

    void addReader(WorkingMemoryReaderComponentPrx reader, Ice.Current __current);

    void receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, Ice.Current __current);
}
