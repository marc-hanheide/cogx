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

public interface _WorkingMemoryDel extends _CASTComponentDel
{
    boolean exists(String id, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.UnknownSubarchitectureException;

    int getVersionNumber(String id, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    cast.cdl.WorkingMemoryPermissions getPermissions(String id, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    boolean tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void unlockEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void setWorkingMemory(WorkingMemoryPrx wm, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException;

    void overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void deleteFromWorkingMemory(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    cast.cdl.WorkingMemoryEntry getWorkingMemoryEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException;

    void getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.UnknownSubarchitectureException;

    void registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void addReader(WorkingMemoryReaderComponentPrx reader, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;

    void receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper;
}
