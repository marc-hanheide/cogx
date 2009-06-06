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

public final class _WorkingMemoryDelM extends Ice._ObjectDelM implements _WorkingMemoryDel
{
    public void
    beat(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("beat", Ice.OperationMode.Idempotent, __ctx);
        try
        {
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    configure(java.util.Map<java.lang.String, java.lang.String> config, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("configure", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                cast.cdl.StringMapHelper.write(__os, config);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    destroy(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("destroy", Ice.OperationMode.Normal, __ctx);
        try
        {
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public String
    getID(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("getID", Ice.OperationMode.Idempotent, __ctx);
        try
        {
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                String __ret;
                __ret = __is.readString();
                __is.endReadEncaps();
                return __ret;
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    run(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("run", Ice.OperationMode.Normal, __ctx);
        try
        {
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    setComponentManager(ComponentManagerPrx man, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("setComponentManager", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                ComponentManagerPrxHelper.__write(__os, man);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    setID(String id, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("setID", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    setTimeServer(TimeServerPrx ts, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("setTimeServer", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                TimeServerPrxHelper.__write(__os, ts);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    start(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("start", Ice.OperationMode.Normal, __ctx);
        try
        {
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    stop(java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("stop", Ice.OperationMode.Normal, __ctx);
        try
        {
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    addReader(WorkingMemoryReaderComponentPrx reader, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("addReader", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                WorkingMemoryReaderComponentPrxHelper.__write(__os, reader);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    addToWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.AlreadyExistsOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("addToWorkingMemory", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(type);
                __os.writeString(component);
                __os.writeObject(entry);
                __os.writePendingObjects();
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.AlreadyExistsOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                __og.is().skipEmptyEncaps();
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    deleteFromWorkingMemory(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("deleteFromWorkingMemory", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(component);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                __og.is().skipEmptyEncaps();
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public boolean
    exists(String id, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("exists", Ice.OperationMode.Idempotent, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                boolean __ret;
                __ret = __is.readBool();
                __is.endReadEncaps();
                return __ret;
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public cast.cdl.WorkingMemoryPermissions
    getPermissions(String id, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("getPermissions", Ice.OperationMode.Idempotent, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                cast.cdl.WorkingMemoryPermissions __ret;
                __ret = cast.cdl.WorkingMemoryPermissions.__read(__is);
                __is.endReadEncaps();
                return __ret;
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public int
    getVersionNumber(String id, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("getVersionNumber", Ice.OperationMode.Idempotent, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                int __ret;
                __ret = __is.readInt();
                __is.endReadEncaps();
                return __ret;
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    getWorkingMemoryEntries(String type, String subarch, int count, String component, cast.cdl.WorkingMemoryEntrySeqHolder entries, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("getWorkingMemoryEntries", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(type);
                __os.writeString(subarch);
                __os.writeInt(count);
                __os.writeString(component);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                entries.value = cast.cdl.WorkingMemoryEntrySeqHelper.read(__is);
                __is.readPendingObjects();
                __is.endReadEncaps();
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public cast.cdl.WorkingMemoryEntry
    getWorkingMemoryEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("getWorkingMemoryEntry", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(component);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                cast.cdl.WorkingMemoryEntryHolder __ret = new cast.cdl.WorkingMemoryEntryHolder();
                __is.readObject(__ret.getPatcher());
                __is.readPendingObjects();
                __is.endReadEncaps();
                return __ret.value;
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    lockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("lockEntry", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(component);
                permissions.__write(__os);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                __og.is().skipEmptyEncaps();
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    overwriteWorkingMemory(String id, String subarch, String type, String component, Ice.Object entry, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("overwriteWorkingMemory", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(type);
                __os.writeString(component);
                __os.writeObject(entry);
                __os.writePendingObjects();
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                __og.is().skipEmptyEncaps();
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    receiveChangeEvent(cast.cdl.WorkingMemoryChange wmc, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("receiveChangeEvent", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                wmc.__write(__os);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    registerComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("registerComponentFilter", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                filter.__write(__os);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    registerWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("registerWorkingMemoryFilter", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                filter.__write(__os);
                __os.writeString(subarch);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    removeComponentFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("removeComponentFilter", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                filter.__write(__os);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    removeWorkingMemoryFilter(cast.cdl.WorkingMemoryChangeFilter filter, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("removeWorkingMemoryFilter", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                filter.__write(__os);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    setWorkingMemory(WorkingMemoryPrx wm, String subarch, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("setWorkingMemory", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                WorkingMemoryPrxHelper.__write(__os, wm);
                __os.writeString(subarch);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            if(!__og.is().isEmpty())
            {
                try
                {
                    if(!__ok)
                    {
                        try
                        {
                            __og.throwUserException();
                        }
                        catch(Ice.UserException __ex)
                        {
                            throw new Ice.UnknownUserException(__ex.ice_name());
                        }
                    }
                    __og.is().skipEmptyEncaps();
                }
                catch(Ice.LocalException __ex)
                {
                    throw new IceInternal.LocalExceptionWrapper(__ex, false);
                }
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public boolean
    tryLockEntry(String id, String subarch, String component, cast.cdl.WorkingMemoryPermissions permissions, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("tryLockEntry", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(component);
                permissions.__write(__os);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                IceInternal.BasicStream __is = __og.is();
                __is.startReadEncaps();
                boolean __ret;
                __ret = __is.readBool();
                __is.endReadEncaps();
                return __ret;
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }

    public void
    unlockEntry(String id, String subarch, String component, java.util.Map<String, String> __ctx)
        throws IceInternal.LocalExceptionWrapper,
               cast.ConsistencyException,
               cast.DoesNotExistOnWMException,
               cast.UnknownSubarchitectureException
    {
        IceInternal.Outgoing __og = __handler.getOutgoing("unlockEntry", Ice.OperationMode.Normal, __ctx);
        try
        {
            try
            {
                IceInternal.BasicStream __os = __og.os();
                __os.writeString(id);
                __os.writeString(subarch);
                __os.writeString(component);
            }
            catch(Ice.LocalException __ex)
            {
                __og.abort(__ex);
            }
            boolean __ok = __og.invoke();
            try
            {
                if(!__ok)
                {
                    try
                    {
                        __og.throwUserException();
                    }
                    catch(cast.ConsistencyException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.DoesNotExistOnWMException __ex)
                    {
                        throw __ex;
                    }
                    catch(cast.UnknownSubarchitectureException __ex)
                    {
                        throw __ex;
                    }
                    catch(Ice.UserException __ex)
                    {
                        throw new Ice.UnknownUserException(__ex.ice_name());
                    }
                }
                __og.is().skipEmptyEncaps();
            }
            catch(Ice.LocalException __ex)
            {
                throw new IceInternal.LocalExceptionWrapper(__ex, false);
            }
        }
        finally
        {
            __handler.reclaimOutgoing(__og);
        }
    }
}
