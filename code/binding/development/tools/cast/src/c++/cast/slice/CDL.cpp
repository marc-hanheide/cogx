// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `CDL.ice'

#include <CDL.hpp>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <IceUtil/Iterator.h>
#include <IceUtil/ScopedArray.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 303
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

static const ::std::string __cast__interfaces__TimeServer__getCASTTime_name = "getCASTTime";

static const ::std::string __cast__interfaces__TimeServer__fromTimeOfDayDouble_name = "fromTimeOfDayDouble";

static const ::std::string __cast__interfaces__TimeServer__fromTimeOfDay_name = "fromTimeOfDay";

static const ::std::string __cast__interfaces__TimeServer__reset_name = "reset";

static const ::std::string __cast__interfaces__CASTComponent__beat_name = "beat";

static const ::std::string __cast__interfaces__CASTComponent__setID_name = "setID";

static const ::std::string __cast__interfaces__CASTComponent__getID_name = "getID";

static const ::std::string __cast__interfaces__CASTComponent__configure_name = "configure";

static const ::std::string __cast__interfaces__CASTComponent__start_name = "start";

static const ::std::string __cast__interfaces__CASTComponent__run_name = "run";

static const ::std::string __cast__interfaces__CASTComponent__stop_name = "stop";

static const ::std::string __cast__interfaces__CASTComponent__setComponentManager_name = "setComponentManager";

static const ::std::string __cast__interfaces__CASTComponent__setTimeServer_name = "setTimeServer";

static const ::std::string __cast__interfaces__CASTComponent__destroy_name = "destroy";

static const ::std::string __cast__interfaces__WorkingMemoryAttachedComponent__setWorkingMemory_name = "setWorkingMemory";

static const ::std::string __cast__interfaces__WorkingMemoryReaderComponent__receiveChangeEvent_name = "receiveChangeEvent";

static const ::std::string __cast__interfaces__ManagedComponent__setTaskManager_name = "setTaskManager";

static const ::std::string __cast__interfaces__ManagedComponent__taskDecision_name = "taskDecision";

static const ::std::string __cast__interfaces__TaskManager__proposeTask_name = "proposeTask";

static const ::std::string __cast__interfaces__TaskManager__retractTask_name = "retractTask";

static const ::std::string __cast__interfaces__TaskManager__taskComplete_name = "taskComplete";

static const ::std::string __cast__interfaces__TaskManager__addManagedComponent_name = "addManagedComponent";

static const ::std::string __cast__interfaces__WorkingMemory__exists_name = "exists";

static const ::std::string __cast__interfaces__WorkingMemory__getVersionNumber_name = "getVersionNumber";

static const ::std::string __cast__interfaces__WorkingMemory__getPermissions_name = "getPermissions";

static const ::std::string __cast__interfaces__WorkingMemory__lockEntry_name = "lockEntry";

static const ::std::string __cast__interfaces__WorkingMemory__tryLockEntry_name = "tryLockEntry";

static const ::std::string __cast__interfaces__WorkingMemory__unlockEntry_name = "unlockEntry";

static const ::std::string __cast__interfaces__WorkingMemory__setWorkingMemory_name = "setWorkingMemory";

static const ::std::string __cast__interfaces__WorkingMemory__addToWorkingMemory_name = "addToWorkingMemory";

static const ::std::string __cast__interfaces__WorkingMemory__overwriteWorkingMemory_name = "overwriteWorkingMemory";

static const ::std::string __cast__interfaces__WorkingMemory__deleteFromWorkingMemory_name = "deleteFromWorkingMemory";

static const ::std::string __cast__interfaces__WorkingMemory__getWorkingMemoryEntry_name = "getWorkingMemoryEntry";

static const ::std::string __cast__interfaces__WorkingMemory__getWorkingMemoryEntries_name = "getWorkingMemoryEntries";

static const ::std::string __cast__interfaces__WorkingMemory__registerComponentFilter_name = "registerComponentFilter";

static const ::std::string __cast__interfaces__WorkingMemory__removeComponentFilter_name = "removeComponentFilter";

static const ::std::string __cast__interfaces__WorkingMemory__registerWorkingMemoryFilter_name = "registerWorkingMemoryFilter";

static const ::std::string __cast__interfaces__WorkingMemory__removeWorkingMemoryFilter_name = "removeWorkingMemoryFilter";

static const ::std::string __cast__interfaces__WorkingMemory__addReader_name = "addReader";

static const ::std::string __cast__interfaces__WorkingMemory__receiveChangeEvent_name = "receiveChangeEvent";

static const ::std::string __cast__interfaces__ComponentManager__getCASTTime_name = "getCASTTime";

static const ::std::string __cast__interfaces__ComponentFactory__newComponent_name = "newComponent";

static const ::std::string __cast__interfaces__ComponentFactory__newManagedComponent_name = "newManagedComponent";

static const ::std::string __cast__interfaces__ComponentFactory__newUnmanagedComponent_name = "newUnmanagedComponent";

static const ::std::string __cast__interfaces__ComponentFactory__newWorkingMemory_name = "newWorkingMemory";

static const ::std::string __cast__interfaces__ComponentFactory__newTaskManager_name = "newTaskManager";

::Ice::Object* IceInternal::upCast(::cast::cdl::TestStructString* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::cdl::TestStructString* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::cdl::TestStructInt* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::cdl::TestStructInt* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::cdl::WorkingMemoryEntry* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::cdl::WorkingMemoryEntry* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::cdl::testing::CASTTestStruct* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::cdl::testing::CASTTestStruct* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::cdl::testing::TestDummyStruct* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::cdl::testing::TestDummyStruct* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::TimeServer* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::TimeServer* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::CASTComponent* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::CASTComponent* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::WorkingMemoryAttachedComponent* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::WorkingMemoryAttachedComponent* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::WorkingMemoryReaderComponent* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::WorkingMemoryReaderComponent* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::ManagedComponent* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::ManagedComponent* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::UnmanagedComponent* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::UnmanagedComponent* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::TaskManager* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::TaskManager* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::WorkingMemory* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::WorkingMemory* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::ComponentManager* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::ComponentManager* p) { return p; }

::Ice::Object* IceInternal::upCast(::cast::interfaces::ComponentFactory* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::cast::interfaces::ComponentFactory* p) { return p; }

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::TestStructStringPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::cdl::TestStructString;
        v->__copyFrom(proxy);
    }
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::TestStructIntPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::cdl::TestStructInt;
        v->__copyFrom(proxy);
    }
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::WorkingMemoryEntryPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::cdl::WorkingMemoryEntry;
        v->__copyFrom(proxy);
    }
}

void
cast::cdl::testing::__read(::IceInternal::BasicStream* __is, ::cast::cdl::testing::CASTTestStructPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::cdl::testing::CASTTestStruct;
        v->__copyFrom(proxy);
    }
}

void
cast::cdl::testing::__read(::IceInternal::BasicStream* __is, ::cast::cdl::testing::TestDummyStructPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::cdl::testing::TestDummyStruct;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::TimeServerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::TimeServer;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::CASTComponentPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::CASTComponent;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::WorkingMemoryAttachedComponentPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::WorkingMemoryAttachedComponent;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::WorkingMemoryReaderComponentPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::WorkingMemoryReaderComponent;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::ManagedComponentPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::ManagedComponent;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::UnmanagedComponentPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::UnmanagedComponent;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::TaskManagerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::TaskManager;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::WorkingMemoryPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::WorkingMemory;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::ComponentManagerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::ComponentManager;
        v->__copyFrom(proxy);
    }
}

void
cast::interfaces::__read(::IceInternal::BasicStream* __is, ::cast::interfaces::ComponentFactoryPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::cast::interfaces::ComponentFactory;
        v->__copyFrom(proxy);
    }
}

void
cast::cdl::__writeStringMap(::IceInternal::BasicStream* __os, const ::cast::cdl::StringMap& v)
{
    __os->writeSize(::Ice::Int(v.size()));
    ::cast::cdl::StringMap::const_iterator p;
    for(p = v.begin(); p != v.end(); ++p)
    {
        __os->write(p->first);
        __os->write(p->second);
    }
}

void
cast::cdl::__readStringMap(::IceInternal::BasicStream* __is, ::cast::cdl::StringMap& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    while(sz--)
    {
        ::std::pair<const  ::std::string, ::std::string> pair;
        __is->read(const_cast< ::std::string&>(pair.first));
        ::cast::cdl::StringMap::iterator __i = v.insert(v.end(), pair);
        __is->read(__i->second);
    }
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::ComponentLanguage v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::ComponentLanguage& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::cast::cdl::ComponentLanguage>(val);
}

bool
cast::cdl::ComponentDescription::operator==(const ComponentDescription& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(componentName != __rhs.componentName)
    {
        return false;
    }
    if(className != __rhs.className)
    {
        return false;
    }
    if(language != __rhs.language)
    {
        return false;
    }
    if(hostName != __rhs.hostName)
    {
        return false;
    }
    if(configuration != __rhs.configuration)
    {
        return false;
    }
    return true;
}

bool
cast::cdl::ComponentDescription::operator<(const ComponentDescription& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(componentName < __rhs.componentName)
    {
        return true;
    }
    else if(__rhs.componentName < componentName)
    {
        return false;
    }
    if(className < __rhs.className)
    {
        return true;
    }
    else if(__rhs.className < className)
    {
        return false;
    }
    if(language < __rhs.language)
    {
        return true;
    }
    else if(__rhs.language < language)
    {
        return false;
    }
    if(hostName < __rhs.hostName)
    {
        return true;
    }
    else if(__rhs.hostName < hostName)
    {
        return false;
    }
    if(configuration < __rhs.configuration)
    {
        return true;
    }
    else if(__rhs.configuration < configuration)
    {
        return false;
    }
    return false;
}

void
cast::cdl::ComponentDescription::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(componentName);
    __os->write(className);
    ::cast::cdl::__write(__os, language);
    __os->write(hostName);
    ::cast::cdl::__writeStringMap(__os, configuration);
}

void
cast::cdl::ComponentDescription::__read(::IceInternal::BasicStream* __is)
{
    __is->read(componentName);
    __is->read(className);
    ::cast::cdl::__read(__is, language);
    __is->read(hostName);
    ::cast::cdl::__readStringMap(__is, configuration);
}

bool
cast::cdl::WorkingMemoryAddress::operator==(const WorkingMemoryAddress& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(id != __rhs.id)
    {
        return false;
    }
    if(subarchitecture != __rhs.subarchitecture)
    {
        return false;
    }
    return true;
}

bool
cast::cdl::WorkingMemoryAddress::operator<(const WorkingMemoryAddress& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(id < __rhs.id)
    {
        return true;
    }
    else if(__rhs.id < id)
    {
        return false;
    }
    if(subarchitecture < __rhs.subarchitecture)
    {
        return true;
    }
    else if(__rhs.subarchitecture < subarchitecture)
    {
        return false;
    }
    return false;
}

void
cast::cdl::WorkingMemoryAddress::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(id);
    __os->write(subarchitecture);
}

void
cast::cdl::WorkingMemoryAddress::__read(::IceInternal::BasicStream* __is)
{
    __is->read(id);
    __is->read(subarchitecture);
}

void
cast::cdl::__writeWorkingMemoryEntrySeq(::IceInternal::BasicStream* __os, const ::cast::cdl::WorkingMemoryEntryPtr* begin, const ::cast::cdl::WorkingMemoryEntryPtr* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(begin[i].get())));
    }
}

void
cast::cdl::__readWorkingMemoryEntrySeq(::IceInternal::BasicStream* __is, ::cast::cdl::WorkingMemoryEntrySeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 4);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        __is->read(::cast::cdl::__patch__WorkingMemoryEntryPtr, &v[i]);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::WorkingMemoryOperation v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 5);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::WorkingMemoryOperation& v)
{
    ::Ice::Byte val;
    __is->read(val, 5);
    v = static_cast< ::cast::cdl::WorkingMemoryOperation>(val);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::WorkingMemoryChangeQueueBehaviour v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::WorkingMemoryChangeQueueBehaviour& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::cast::cdl::WorkingMemoryChangeQueueBehaviour>(val);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::WorkingMemoryPermissions v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 6);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::WorkingMemoryPermissions& v)
{
    ::Ice::Byte val;
    __is->read(val, 6);
    v = static_cast< ::cast::cdl::WorkingMemoryPermissions>(val);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::WorkingMemoryLockRequest v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 8);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::WorkingMemoryLockRequest& v)
{
    ::Ice::Byte val;
    __is->read(val, 8);
    v = static_cast< ::cast::cdl::WorkingMemoryLockRequest>(val);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::FilterRestriction v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::FilterRestriction& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::cast::cdl::FilterRestriction>(val);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::ReceiverDeleteCondition v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::ReceiverDeleteCondition& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::cast::cdl::ReceiverDeleteCondition>(val);
}

bool
cast::cdl::WorkingMemoryChange::operator==(const WorkingMemoryChange& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(operation != __rhs.operation)
    {
        return false;
    }
    if(src != __rhs.src)
    {
        return false;
    }
    if(address != __rhs.address)
    {
        return false;
    }
    if(type != __rhs.type)
    {
        return false;
    }
    return true;
}

bool
cast::cdl::WorkingMemoryChange::operator<(const WorkingMemoryChange& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(operation < __rhs.operation)
    {
        return true;
    }
    else if(__rhs.operation < operation)
    {
        return false;
    }
    if(src < __rhs.src)
    {
        return true;
    }
    else if(__rhs.src < src)
    {
        return false;
    }
    if(address < __rhs.address)
    {
        return true;
    }
    else if(__rhs.address < address)
    {
        return false;
    }
    if(type < __rhs.type)
    {
        return true;
    }
    else if(__rhs.type < type)
    {
        return false;
    }
    return false;
}

void
cast::cdl::WorkingMemoryChange::__write(::IceInternal::BasicStream* __os) const
{
    ::cast::cdl::__write(__os, operation);
    __os->write(src);
    address.__write(__os);
    __os->write(type);
}

void
cast::cdl::WorkingMemoryChange::__read(::IceInternal::BasicStream* __is)
{
    ::cast::cdl::__read(__is, operation);
    __is->read(src);
    address.__read(__is);
    __is->read(type);
}

bool
cast::cdl::WorkingMemoryChangeFilter::operator==(const WorkingMemoryChangeFilter& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(operation != __rhs.operation)
    {
        return false;
    }
    if(src != __rhs.src)
    {
        return false;
    }
    if(address != __rhs.address)
    {
        return false;
    }
    if(type != __rhs.type)
    {
        return false;
    }
    if(restriction != __rhs.restriction)
    {
        return false;
    }
    if(origin != __rhs.origin)
    {
        return false;
    }
    return true;
}

bool
cast::cdl::WorkingMemoryChangeFilter::operator<(const WorkingMemoryChangeFilter& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(operation < __rhs.operation)
    {
        return true;
    }
    else if(__rhs.operation < operation)
    {
        return false;
    }
    if(src < __rhs.src)
    {
        return true;
    }
    else if(__rhs.src < src)
    {
        return false;
    }
    if(address < __rhs.address)
    {
        return true;
    }
    else if(__rhs.address < address)
    {
        return false;
    }
    if(type < __rhs.type)
    {
        return true;
    }
    else if(__rhs.type < type)
    {
        return false;
    }
    if(restriction < __rhs.restriction)
    {
        return true;
    }
    else if(__rhs.restriction < restriction)
    {
        return false;
    }
    if(origin < __rhs.origin)
    {
        return true;
    }
    else if(__rhs.origin < origin)
    {
        return false;
    }
    return false;
}

void
cast::cdl::WorkingMemoryChangeFilter::__write(::IceInternal::BasicStream* __os) const
{
    ::cast::cdl::__write(__os, operation);
    __os->write(src);
    address.__write(__os);
    __os->write(type);
    ::cast::cdl::__write(__os, restriction);
    __os->write(origin);
}

void
cast::cdl::WorkingMemoryChangeFilter::__read(::IceInternal::BasicStream* __is)
{
    ::cast::cdl::__read(__is, operation);
    __is->read(src);
    address.__read(__is);
    __is->read(type);
    ::cast::cdl::__read(__is, restriction);
    __is->read(origin);
}

bool
cast::cdl::CASTTime::operator==(const CASTTime& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(s != __rhs.s)
    {
        return false;
    }
    if(us != __rhs.us)
    {
        return false;
    }
    return true;
}

bool
cast::cdl::CASTTime::operator<(const CASTTime& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(s < __rhs.s)
    {
        return true;
    }
    else if(__rhs.s < s)
    {
        return false;
    }
    if(us < __rhs.us)
    {
        return true;
    }
    else if(__rhs.us < us)
    {
        return false;
    }
    return false;
}

void
cast::cdl::CASTTime::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(s);
    __os->write(us);
}

void
cast::cdl::CASTTime::__read(::IceInternal::BasicStream* __is)
{
    __is->read(s);
    __is->read(us);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::TaskOutcome v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 4);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::TaskOutcome& v)
{
    ::Ice::Byte val;
    __is->read(val, 4);
    v = static_cast< ::cast::cdl::TaskOutcome>(val);
}

void
cast::cdl::__write(::IceInternal::BasicStream* __os, ::cast::cdl::TaskManagementDecision v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 3);
}

void
cast::cdl::__read(::IceInternal::BasicStream* __is, ::cast::cdl::TaskManagementDecision& v)
{
    ::Ice::Byte val;
    __is->read(val, 3);
    v = static_cast< ::cast::cdl::TaskManagementDecision>(val);
}

cast::CASTException::CASTException(const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    UserException(),
#else
    ::Ice::UserException(),
#endif
    message(__ice_message)
{
}

cast::CASTException::~CASTException() throw()
{
}

static const char* __cast__CASTException_name = "cast::CASTException";

::std::string
cast::CASTException::ice_name() const
{
    return __cast__CASTException_name;
}

::Ice::Exception*
cast::CASTException::ice_clone() const
{
    return new CASTException(*this);
}

void
cast::CASTException::ice_throw() const
{
    throw *this;
}

void
cast::CASTException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::CASTException"), false);
    __os->startWriteSlice();
    __os->write(message);
    __os->endWriteSlice();
}

void
cast::CASTException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->read(message);
    __is->endReadSlice();
}

void
cast::CASTException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::CASTException was not generated with stream support";
    throw ex;
}

void
cast::CASTException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::CASTException was not generated with stream support";
    throw ex;
}

struct __F__cast__CASTException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::CASTException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__CASTException__Ptr = new __F__cast__CASTException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::CASTException::ice_factory()
{
    return __F__cast__CASTException__Ptr;
}

class __F__cast__CASTException__Init
{
public:

    __F__cast__CASTException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::CASTException", ::cast::CASTException::ice_factory());
    }

    ~__F__cast__CASTException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::CASTException");
    }
};

static __F__cast__CASTException__Init __F__cast__CASTException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__CASTException__initializer() {} }
#endif

cast::ComponentCreationException::ComponentCreationException(const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException(__ice_message)
#else
    ::cast::CASTException(__ice_message)
#endif
{
}

cast::ComponentCreationException::~ComponentCreationException() throw()
{
}

static const char* __cast__ComponentCreationException_name = "cast::ComponentCreationException";

::std::string
cast::ComponentCreationException::ice_name() const
{
    return __cast__ComponentCreationException_name;
}

::Ice::Exception*
cast::ComponentCreationException::ice_clone() const
{
    return new ComponentCreationException(*this);
}

void
cast::ComponentCreationException::ice_throw() const
{
    throw *this;
}

void
cast::ComponentCreationException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::ComponentCreationException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__write(__os);
#else
    ::cast::CASTException::__write(__os);
#endif
}

void
cast::ComponentCreationException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__read(__is, true);
#else
    ::cast::CASTException::__read(__is, true);
#endif
}

void
cast::ComponentCreationException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::ComponentCreationException was not generated with stream support";
    throw ex;
}

void
cast::ComponentCreationException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::ComponentCreationException was not generated with stream support";
    throw ex;
}

struct __F__cast__ComponentCreationException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::ComponentCreationException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__ComponentCreationException__Ptr = new __F__cast__ComponentCreationException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::ComponentCreationException::ice_factory()
{
    return __F__cast__ComponentCreationException__Ptr;
}

class __F__cast__ComponentCreationException__Init
{
public:

    __F__cast__ComponentCreationException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::ComponentCreationException", ::cast::ComponentCreationException::ice_factory());
    }

    ~__F__cast__ComponentCreationException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::ComponentCreationException");
    }
};

static __F__cast__ComponentCreationException__Init __F__cast__ComponentCreationException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__ComponentCreationException__initializer() {} }
#endif

cast::SubarchitectureComponentException::SubarchitectureComponentException(const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException(__ice_message)
#else
    ::cast::CASTException(__ice_message)
#endif
{
}

cast::SubarchitectureComponentException::~SubarchitectureComponentException() throw()
{
}

static const char* __cast__SubarchitectureComponentException_name = "cast::SubarchitectureComponentException";

::std::string
cast::SubarchitectureComponentException::ice_name() const
{
    return __cast__SubarchitectureComponentException_name;
}

::Ice::Exception*
cast::SubarchitectureComponentException::ice_clone() const
{
    return new SubarchitectureComponentException(*this);
}

void
cast::SubarchitectureComponentException::ice_throw() const
{
    throw *this;
}

void
cast::SubarchitectureComponentException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::SubarchitectureComponentException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__write(__os);
#else
    ::cast::CASTException::__write(__os);
#endif
}

void
cast::SubarchitectureComponentException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__read(__is, true);
#else
    ::cast::CASTException::__read(__is, true);
#endif
}

void
cast::SubarchitectureComponentException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::SubarchitectureComponentException was not generated with stream support";
    throw ex;
}

void
cast::SubarchitectureComponentException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::SubarchitectureComponentException was not generated with stream support";
    throw ex;
}

struct __F__cast__SubarchitectureComponentException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::SubarchitectureComponentException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__SubarchitectureComponentException__Ptr = new __F__cast__SubarchitectureComponentException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::SubarchitectureComponentException::ice_factory()
{
    return __F__cast__SubarchitectureComponentException__Ptr;
}

class __F__cast__SubarchitectureComponentException__Init
{
public:

    __F__cast__SubarchitectureComponentException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::SubarchitectureComponentException", ::cast::SubarchitectureComponentException::ice_factory());
    }

    ~__F__cast__SubarchitectureComponentException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::SubarchitectureComponentException");
    }
};

static __F__cast__SubarchitectureComponentException__Init __F__cast__SubarchitectureComponentException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__SubarchitectureComponentException__initializer() {} }
#endif

cast::UnknownSubarchitectureException::UnknownSubarchitectureException(const ::std::string& __ice_message, const ::std::string& __ice_subarchitecture) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    SubarchitectureComponentException(__ice_message),
#else
    ::cast::SubarchitectureComponentException(__ice_message),
#endif
    subarchitecture(__ice_subarchitecture)
{
}

cast::UnknownSubarchitectureException::~UnknownSubarchitectureException() throw()
{
}

static const char* __cast__UnknownSubarchitectureException_name = "cast::UnknownSubarchitectureException";

::std::string
cast::UnknownSubarchitectureException::ice_name() const
{
    return __cast__UnknownSubarchitectureException_name;
}

::Ice::Exception*
cast::UnknownSubarchitectureException::ice_clone() const
{
    return new UnknownSubarchitectureException(*this);
}

void
cast::UnknownSubarchitectureException::ice_throw() const
{
    throw *this;
}

void
cast::UnknownSubarchitectureException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::UnknownSubarchitectureException"), false);
    __os->startWriteSlice();
    __os->write(subarchitecture);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    SubarchitectureComponentException::__write(__os);
#else
    ::cast::SubarchitectureComponentException::__write(__os);
#endif
}

void
cast::UnknownSubarchitectureException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->read(subarchitecture);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    SubarchitectureComponentException::__read(__is, true);
#else
    ::cast::SubarchitectureComponentException::__read(__is, true);
#endif
}

void
cast::UnknownSubarchitectureException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::UnknownSubarchitectureException was not generated with stream support";
    throw ex;
}

void
cast::UnknownSubarchitectureException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::UnknownSubarchitectureException was not generated with stream support";
    throw ex;
}

struct __F__cast__UnknownSubarchitectureException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::UnknownSubarchitectureException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__UnknownSubarchitectureException__Ptr = new __F__cast__UnknownSubarchitectureException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::UnknownSubarchitectureException::ice_factory()
{
    return __F__cast__UnknownSubarchitectureException__Ptr;
}

class __F__cast__UnknownSubarchitectureException__Init
{
public:

    __F__cast__UnknownSubarchitectureException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::UnknownSubarchitectureException", ::cast::UnknownSubarchitectureException::ice_factory());
    }

    ~__F__cast__UnknownSubarchitectureException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::UnknownSubarchitectureException");
    }
};

static __F__cast__UnknownSubarchitectureException__Init __F__cast__UnknownSubarchitectureException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__UnknownSubarchitectureException__initializer() {} }
#endif

cast::WMException::WMException(const ::std::string& __ice_message, const ::cast::cdl::WorkingMemoryAddress& __ice_wma) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    SubarchitectureComponentException(__ice_message),
#else
    ::cast::SubarchitectureComponentException(__ice_message),
#endif
    wma(__ice_wma)
{
}

cast::WMException::~WMException() throw()
{
}

static const char* __cast__WMException_name = "cast::WMException";

::std::string
cast::WMException::ice_name() const
{
    return __cast__WMException_name;
}

::Ice::Exception*
cast::WMException::ice_clone() const
{
    return new WMException(*this);
}

void
cast::WMException::ice_throw() const
{
    throw *this;
}

void
cast::WMException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::WMException"), false);
    __os->startWriteSlice();
    wma.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    SubarchitectureComponentException::__write(__os);
#else
    ::cast::SubarchitectureComponentException::__write(__os);
#endif
}

void
cast::WMException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    wma.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    SubarchitectureComponentException::__read(__is, true);
#else
    ::cast::SubarchitectureComponentException::__read(__is, true);
#endif
}

void
cast::WMException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::WMException was not generated with stream support";
    throw ex;
}

void
cast::WMException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::WMException was not generated with stream support";
    throw ex;
}

struct __F__cast__WMException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::WMException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__WMException__Ptr = new __F__cast__WMException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::WMException::ice_factory()
{
    return __F__cast__WMException__Ptr;
}

class __F__cast__WMException__Init
{
public:

    __F__cast__WMException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::WMException", ::cast::WMException::ice_factory());
    }

    ~__F__cast__WMException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::WMException");
    }
};

static __F__cast__WMException__Init __F__cast__WMException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__WMException__initializer() {} }
#endif

cast::DoesNotExistOnWMException::DoesNotExistOnWMException(const ::std::string& __ice_message, const ::cast::cdl::WorkingMemoryAddress& __ice_wma) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException(__ice_message, __ice_wma)
#else
    ::cast::WMException(__ice_message, __ice_wma)
#endif
{
}

cast::DoesNotExistOnWMException::~DoesNotExistOnWMException() throw()
{
}

static const char* __cast__DoesNotExistOnWMException_name = "cast::DoesNotExistOnWMException";

::std::string
cast::DoesNotExistOnWMException::ice_name() const
{
    return __cast__DoesNotExistOnWMException_name;
}

::Ice::Exception*
cast::DoesNotExistOnWMException::ice_clone() const
{
    return new DoesNotExistOnWMException(*this);
}

void
cast::DoesNotExistOnWMException::ice_throw() const
{
    throw *this;
}

void
cast::DoesNotExistOnWMException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::DoesNotExistOnWMException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__write(__os);
#else
    ::cast::WMException::__write(__os);
#endif
}

void
cast::DoesNotExistOnWMException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__read(__is, true);
#else
    ::cast::WMException::__read(__is, true);
#endif
}

void
cast::DoesNotExistOnWMException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::DoesNotExistOnWMException was not generated with stream support";
    throw ex;
}

void
cast::DoesNotExistOnWMException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::DoesNotExistOnWMException was not generated with stream support";
    throw ex;
}

struct __F__cast__DoesNotExistOnWMException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::DoesNotExistOnWMException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__DoesNotExistOnWMException__Ptr = new __F__cast__DoesNotExistOnWMException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::DoesNotExistOnWMException::ice_factory()
{
    return __F__cast__DoesNotExistOnWMException__Ptr;
}

class __F__cast__DoesNotExistOnWMException__Init
{
public:

    __F__cast__DoesNotExistOnWMException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::DoesNotExistOnWMException", ::cast::DoesNotExistOnWMException::ice_factory());
    }

    ~__F__cast__DoesNotExistOnWMException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::DoesNotExistOnWMException");
    }
};

static __F__cast__DoesNotExistOnWMException__Init __F__cast__DoesNotExistOnWMException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__DoesNotExistOnWMException__initializer() {} }
#endif

cast::AlreadyExistsOnWMException::AlreadyExistsOnWMException(const ::std::string& __ice_message, const ::cast::cdl::WorkingMemoryAddress& __ice_wma) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException(__ice_message, __ice_wma)
#else
    ::cast::WMException(__ice_message, __ice_wma)
#endif
{
}

cast::AlreadyExistsOnWMException::~AlreadyExistsOnWMException() throw()
{
}

static const char* __cast__AlreadyExistsOnWMException_name = "cast::AlreadyExistsOnWMException";

::std::string
cast::AlreadyExistsOnWMException::ice_name() const
{
    return __cast__AlreadyExistsOnWMException_name;
}

::Ice::Exception*
cast::AlreadyExistsOnWMException::ice_clone() const
{
    return new AlreadyExistsOnWMException(*this);
}

void
cast::AlreadyExistsOnWMException::ice_throw() const
{
    throw *this;
}

void
cast::AlreadyExistsOnWMException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::AlreadyExistsOnWMException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__write(__os);
#else
    ::cast::WMException::__write(__os);
#endif
}

void
cast::AlreadyExistsOnWMException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__read(__is, true);
#else
    ::cast::WMException::__read(__is, true);
#endif
}

void
cast::AlreadyExistsOnWMException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::AlreadyExistsOnWMException was not generated with stream support";
    throw ex;
}

void
cast::AlreadyExistsOnWMException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::AlreadyExistsOnWMException was not generated with stream support";
    throw ex;
}

struct __F__cast__AlreadyExistsOnWMException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::AlreadyExistsOnWMException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__AlreadyExistsOnWMException__Ptr = new __F__cast__AlreadyExistsOnWMException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::AlreadyExistsOnWMException::ice_factory()
{
    return __F__cast__AlreadyExistsOnWMException__Ptr;
}

class __F__cast__AlreadyExistsOnWMException__Init
{
public:

    __F__cast__AlreadyExistsOnWMException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::AlreadyExistsOnWMException", ::cast::AlreadyExistsOnWMException::ice_factory());
    }

    ~__F__cast__AlreadyExistsOnWMException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::AlreadyExistsOnWMException");
    }
};

static __F__cast__AlreadyExistsOnWMException__Init __F__cast__AlreadyExistsOnWMException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__AlreadyExistsOnWMException__initializer() {} }
#endif

cast::ConsistencyException::ConsistencyException(const ::std::string& __ice_message, const ::cast::cdl::WorkingMemoryAddress& __ice_wma) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException(__ice_message, __ice_wma)
#else
    ::cast::WMException(__ice_message, __ice_wma)
#endif
{
}

cast::ConsistencyException::~ConsistencyException() throw()
{
}

static const char* __cast__ConsistencyException_name = "cast::ConsistencyException";

::std::string
cast::ConsistencyException::ice_name() const
{
    return __cast__ConsistencyException_name;
}

::Ice::Exception*
cast::ConsistencyException::ice_clone() const
{
    return new ConsistencyException(*this);
}

void
cast::ConsistencyException::ice_throw() const
{
    throw *this;
}

void
cast::ConsistencyException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::ConsistencyException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__write(__os);
#else
    ::cast::WMException::__write(__os);
#endif
}

void
cast::ConsistencyException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__read(__is, true);
#else
    ::cast::WMException::__read(__is, true);
#endif
}

void
cast::ConsistencyException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::ConsistencyException was not generated with stream support";
    throw ex;
}

void
cast::ConsistencyException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::ConsistencyException was not generated with stream support";
    throw ex;
}

struct __F__cast__ConsistencyException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::ConsistencyException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__ConsistencyException__Ptr = new __F__cast__ConsistencyException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::ConsistencyException::ice_factory()
{
    return __F__cast__ConsistencyException__Ptr;
}

class __F__cast__ConsistencyException__Init
{
public:

    __F__cast__ConsistencyException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::ConsistencyException", ::cast::ConsistencyException::ice_factory());
    }

    ~__F__cast__ConsistencyException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::ConsistencyException");
    }
};

static __F__cast__ConsistencyException__Init __F__cast__ConsistencyException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__ConsistencyException__initializer() {} }
#endif

cast::PermissionException::PermissionException(const ::std::string& __ice_message, const ::cast::cdl::WorkingMemoryAddress& __ice_wma) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException(__ice_message, __ice_wma)
#else
    ::cast::WMException(__ice_message, __ice_wma)
#endif
{
}

cast::PermissionException::~PermissionException() throw()
{
}

static const char* __cast__PermissionException_name = "cast::PermissionException";

::std::string
cast::PermissionException::ice_name() const
{
    return __cast__PermissionException_name;
}

::Ice::Exception*
cast::PermissionException::ice_clone() const
{
    return new PermissionException(*this);
}

void
cast::PermissionException::ice_throw() const
{
    throw *this;
}

void
cast::PermissionException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::cast::PermissionException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__write(__os);
#else
    ::cast::WMException::__write(__os);
#endif
}

void
cast::PermissionException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    WMException::__read(__is, true);
#else
    ::cast::WMException::__read(__is, true);
#endif
}

void
cast::PermissionException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::PermissionException was not generated with stream support";
    throw ex;
}

void
cast::PermissionException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception cast::PermissionException was not generated with stream support";
    throw ex;
}

struct __F__cast__PermissionException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::cast::PermissionException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__cast__PermissionException__Ptr = new __F__cast__PermissionException;

const ::IceInternal::UserExceptionFactoryPtr&
cast::PermissionException::ice_factory()
{
    return __F__cast__PermissionException__Ptr;
}

class __F__cast__PermissionException__Init
{
public:

    __F__cast__PermissionException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::cast::PermissionException", ::cast::PermissionException::ice_factory());
    }

    ~__F__cast__PermissionException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::cast::PermissionException");
    }
};

static __F__cast__PermissionException__Init __F__cast__PermissionException__i;

#ifdef __APPLE__
extern "C" { void __F__cast__PermissionException__initializer() {} }
#endif

const ::std::string&
IceProxy::cast::cdl::TestStructString::ice_staticId()
{
    return ::cast::cdl::TestStructString::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::cdl::TestStructString::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::cdl::TestStructString);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::cdl::TestStructString::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::cdl::TestStructString);
}

::IceProxy::Ice::Object*
IceProxy::cast::cdl::TestStructString::__newInstance() const
{
    return new TestStructString;
}

const ::std::string&
IceProxy::cast::cdl::TestStructInt::ice_staticId()
{
    return ::cast::cdl::TestStructInt::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::cdl::TestStructInt::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::cdl::TestStructInt);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::cdl::TestStructInt::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::cdl::TestStructInt);
}

::IceProxy::Ice::Object*
IceProxy::cast::cdl::TestStructInt::__newInstance() const
{
    return new TestStructInt;
}

const ::std::string&
IceProxy::cast::cdl::WorkingMemoryEntry::ice_staticId()
{
    return ::cast::cdl::WorkingMemoryEntry::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::cdl::WorkingMemoryEntry::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::cdl::WorkingMemoryEntry);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::cdl::WorkingMemoryEntry::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::cdl::WorkingMemoryEntry);
}

::IceProxy::Ice::Object*
IceProxy::cast::cdl::WorkingMemoryEntry::__newInstance() const
{
    return new WorkingMemoryEntry;
}

const ::std::string&
IceProxy::cast::cdl::testing::CASTTestStruct::ice_staticId()
{
    return ::cast::cdl::testing::CASTTestStruct::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::cdl::testing::CASTTestStruct::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::cdl::testing::CASTTestStruct);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::cdl::testing::CASTTestStruct::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::cdl::testing::CASTTestStruct);
}

::IceProxy::Ice::Object*
IceProxy::cast::cdl::testing::CASTTestStruct::__newInstance() const
{
    return new CASTTestStruct;
}

const ::std::string&
IceProxy::cast::cdl::testing::TestDummyStruct::ice_staticId()
{
    return ::cast::cdl::testing::TestDummyStruct::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::cdl::testing::TestDummyStruct::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::cdl::testing::TestDummyStruct);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::cdl::testing::TestDummyStruct::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::cdl::testing::TestDummyStruct);
}

::IceProxy::Ice::Object*
IceProxy::cast::cdl::testing::TestDummyStruct::__newInstance() const
{
    return new TestDummyStruct;
}

::cast::cdl::CASTTime
IceProxy::cast::interfaces::TimeServer::getCASTTime(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__TimeServer__getCASTTime_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TimeServer* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TimeServer*>(__delBase.get());
            return __del->getCASTTime(__ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::cdl::CASTTime
IceProxy::cast::interfaces::TimeServer::fromTimeOfDayDouble(::Ice::Double todsecs, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__TimeServer__fromTimeOfDayDouble_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TimeServer* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TimeServer*>(__delBase.get());
            return __del->fromTimeOfDayDouble(todsecs, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::cdl::CASTTime
IceProxy::cast::interfaces::TimeServer::fromTimeOfDay(::Ice::Long secs, ::Ice::Long usecs, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__TimeServer__fromTimeOfDay_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TimeServer* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TimeServer*>(__delBase.get());
            return __del->fromTimeOfDay(secs, usecs, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::TimeServer::reset(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TimeServer* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TimeServer*>(__delBase.get());
            __del->reset(__ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::TimeServer::ice_staticId()
{
    return ::cast::interfaces::TimeServer::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::TimeServer::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::TimeServer);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::TimeServer::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::TimeServer);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::TimeServer::__newInstance() const
{
    return new TimeServer;
}

void
IceProxy::cast::interfaces::CASTComponent::beat(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->beat(__ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapperRelaxed(__delBase, __ex, 0, __cnt);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::setID(const ::std::string& id, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->setID(id, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::std::string
IceProxy::cast::interfaces::CASTComponent::getID(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__CASTComponent__getID_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            return __del->getID(__ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapperRelaxed(__delBase, __ex, 0, __cnt);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::configure(const ::cast::cdl::StringMap& config, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->configure(config, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::start(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->start(__ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::run(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->run(__ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::stop(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->stop(__ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::setComponentManager(const ::cast::interfaces::ComponentManagerPrx& man, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->setComponentManager(man, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::setTimeServer(const ::cast::interfaces::TimeServerPrx& ts, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->setTimeServer(ts, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::CASTComponent::destroy(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::CASTComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::CASTComponent*>(__delBase.get());
            __del->destroy(__ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::CASTComponent::ice_staticId()
{
    return ::cast::interfaces::CASTComponent::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::CASTComponent::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::CASTComponent);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::CASTComponent::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::CASTComponent);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::CASTComponent::__newInstance() const
{
    return new CASTComponent;
}

void
IceProxy::cast::interfaces::WorkingMemoryAttachedComponent::setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemoryAttachedComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemoryAttachedComponent*>(__delBase.get());
            __del->setWorkingMemory(wm, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::WorkingMemoryAttachedComponent::ice_staticId()
{
    return ::cast::interfaces::WorkingMemoryAttachedComponent::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::WorkingMemoryAttachedComponent::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::WorkingMemoryAttachedComponent);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::WorkingMemoryAttachedComponent::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::WorkingMemoryAttachedComponent);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::WorkingMemoryAttachedComponent::__newInstance() const
{
    return new WorkingMemoryAttachedComponent;
}

void
IceProxy::cast::interfaces::WorkingMemoryReaderComponent::receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemoryReaderComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemoryReaderComponent*>(__delBase.get());
            __del->receiveChangeEvent(wmc, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::WorkingMemoryReaderComponent::ice_staticId()
{
    return ::cast::interfaces::WorkingMemoryReaderComponent::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::WorkingMemoryReaderComponent::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::WorkingMemoryReaderComponent);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::WorkingMemoryReaderComponent::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::WorkingMemoryReaderComponent);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::WorkingMemoryReaderComponent::__newInstance() const
{
    return new WorkingMemoryReaderComponent;
}

void
IceProxy::cast::interfaces::ManagedComponent::setTaskManager(const ::cast::interfaces::TaskManagerPrx& tm, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ManagedComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ManagedComponent*>(__delBase.get());
            __del->setTaskManager(tm, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::ManagedComponent::taskDecision(const ::std::string& id, ::cast::cdl::TaskManagementDecision decision, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ManagedComponent* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ManagedComponent*>(__delBase.get());
            __del->taskDecision(id, decision, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::ManagedComponent::ice_staticId()
{
    return ::cast::interfaces::ManagedComponent::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::ManagedComponent::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::ManagedComponent);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::ManagedComponent::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::ManagedComponent);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::ManagedComponent::__newInstance() const
{
    return new ManagedComponent;
}

const ::std::string&
IceProxy::cast::interfaces::UnmanagedComponent::ice_staticId()
{
    return ::cast::interfaces::UnmanagedComponent::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::UnmanagedComponent::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::UnmanagedComponent);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::UnmanagedComponent::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::UnmanagedComponent);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::UnmanagedComponent::__newInstance() const
{
    return new UnmanagedComponent;
}

void
IceProxy::cast::interfaces::TaskManager::proposeTask(const ::std::string& component, const ::std::string& taskID, const ::std::string& taskName, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TaskManager* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TaskManager*>(__delBase.get());
            __del->proposeTask(component, taskID, taskName, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::TaskManager::retractTask(const ::std::string& component, const ::std::string& taskID, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TaskManager* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TaskManager*>(__delBase.get());
            __del->retractTask(component, taskID, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::TaskManager::taskComplete(const ::std::string& component, const ::std::string& taskID, ::cast::cdl::TaskOutcome outcome, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TaskManager* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TaskManager*>(__delBase.get());
            __del->taskComplete(component, taskID, outcome, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::TaskManager::addManagedComponent(const ::cast::interfaces::ManagedComponentPrx& comp, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::TaskManager* __del = dynamic_cast< ::IceDelegate::cast::interfaces::TaskManager*>(__delBase.get());
            __del->addManagedComponent(comp, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::TaskManager::ice_staticId()
{
    return ::cast::interfaces::TaskManager::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::TaskManager::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::TaskManager);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::TaskManager::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::TaskManager);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::TaskManager::__newInstance() const
{
    return new TaskManager;
}

bool
IceProxy::cast::interfaces::WorkingMemory::exists(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__exists_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            return __del->exists(id, subarch, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapperRelaxed(__delBase, __ex, 0, __cnt);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::Ice::Int
IceProxy::cast::interfaces::WorkingMemory::getVersionNumber(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__getVersionNumber_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            return __del->getVersionNumber(id, subarch, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapperRelaxed(__delBase, __ex, 0, __cnt);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::cdl::WorkingMemoryPermissions
IceProxy::cast::interfaces::WorkingMemory::getPermissions(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__getPermissions_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            return __del->getPermissions(id, subarch, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapperRelaxed(__delBase, __ex, 0, __cnt);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::lockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__lockEntry_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->lockEntry(id, subarch, component, permissions, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

bool
IceProxy::cast::interfaces::WorkingMemory::tryLockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__tryLockEntry_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            return __del->tryLockEntry(id, subarch, component, permissions, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::unlockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__unlockEntry_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->unlockEntry(id, subarch, component, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::std::string& subarch, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->setWorkingMemory(wm, subarch, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::addToWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__addToWorkingMemory_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->addToWorkingMemory(id, subarch, type, component, entry, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::overwriteWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__overwriteWorkingMemory_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->overwriteWorkingMemory(id, subarch, type, component, entry, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::deleteFromWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__deleteFromWorkingMemory_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->deleteFromWorkingMemory(id, subarch, component, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::cdl::WorkingMemoryEntryPtr
IceProxy::cast::interfaces::WorkingMemory::getWorkingMemoryEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__getWorkingMemoryEntry_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            return __del->getWorkingMemoryEntry(id, subarch, component, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::getWorkingMemoryEntries(const ::std::string& type, const ::std::string& subarch, ::Ice::Int count, const ::std::string& component, ::cast::cdl::WorkingMemoryEntrySeq& entries, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__WorkingMemory__getWorkingMemoryEntries_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->getWorkingMemoryEntries(type, subarch, count, component, entries, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->registerComponentFilter(filter, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->removeComponentFilter(filter, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::std::string& subarch, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->registerWorkingMemoryFilter(filter, subarch, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->removeWorkingMemoryFilter(filter, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx& reader, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->addReader(reader, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

void
IceProxy::cast::interfaces::WorkingMemory::receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::WorkingMemory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::WorkingMemory*>(__delBase.get());
            __del->receiveChangeEvent(wmc, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::WorkingMemory::ice_staticId()
{
    return ::cast::interfaces::WorkingMemory::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::WorkingMemory::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::WorkingMemory);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::WorkingMemory::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::WorkingMemory);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::WorkingMemory::__newInstance() const
{
    return new WorkingMemory;
}

::cast::cdl::CASTTime
IceProxy::cast::interfaces::ComponentManager::getCASTTime(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__ComponentManager__getCASTTime_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ComponentManager* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ComponentManager*>(__delBase.get());
            return __del->getCASTTime(__ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::ComponentManager::ice_staticId()
{
    return ::cast::interfaces::ComponentManager::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::ComponentManager::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::ComponentManager);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::ComponentManager::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::ComponentManager);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::ComponentManager::__newInstance() const
{
    return new ComponentManager;
}

::cast::interfaces::CASTComponentPrx
IceProxy::cast::interfaces::ComponentFactory::newComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__ComponentFactory__newComponent_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ComponentFactory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ComponentFactory*>(__delBase.get());
            return __del->newComponent(id, type, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::interfaces::ManagedComponentPrx
IceProxy::cast::interfaces::ComponentFactory::newManagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__ComponentFactory__newManagedComponent_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ComponentFactory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ComponentFactory*>(__delBase.get());
            return __del->newManagedComponent(id, type, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::interfaces::UnmanagedComponentPrx
IceProxy::cast::interfaces::ComponentFactory::newUnmanagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__ComponentFactory__newUnmanagedComponent_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ComponentFactory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ComponentFactory*>(__delBase.get());
            return __del->newUnmanagedComponent(id, type, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::interfaces::WorkingMemoryPrx
IceProxy::cast::interfaces::ComponentFactory::newWorkingMemory(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__ComponentFactory__newWorkingMemory_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ComponentFactory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ComponentFactory*>(__delBase.get());
            return __del->newWorkingMemory(id, type, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

::cast::interfaces::TaskManagerPrx
IceProxy::cast::interfaces::ComponentFactory::newTaskManager(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__cast__interfaces__ComponentFactory__newTaskManager_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::cast::interfaces::ComponentFactory* __del = dynamic_cast< ::IceDelegate::cast::interfaces::ComponentFactory*>(__delBase.get());
            return __del->newTaskManager(id, type, __ctx);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::cast::interfaces::ComponentFactory::ice_staticId()
{
    return ::cast::interfaces::ComponentFactory::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::cast::interfaces::ComponentFactory::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::cast::interfaces::ComponentFactory);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::cast::interfaces::ComponentFactory::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::cast::interfaces::ComponentFactory);
}

::IceProxy::Ice::Object*
IceProxy::cast::interfaces::ComponentFactory::__newInstance() const
{
    return new ComponentFactory;
}

::cast::cdl::CASTTime
IceDelegateM::cast::interfaces::TimeServer::getCASTTime(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TimeServer__getCASTTime_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::cdl::CASTTime __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __ret.__read(__is);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::cdl::CASTTime
IceDelegateM::cast::interfaces::TimeServer::fromTimeOfDayDouble(::Ice::Double todsecs, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TimeServer__fromTimeOfDayDouble_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(todsecs);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::cdl::CASTTime __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __ret.__read(__is);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::cdl::CASTTime
IceDelegateM::cast::interfaces::TimeServer::fromTimeOfDay(::Ice::Long secs, ::Ice::Long usecs, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TimeServer__fromTimeOfDay_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(secs);
        __os->write(usecs);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::cdl::CASTTime __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __ret.__read(__is);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::TimeServer::reset(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TimeServer__reset_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::beat(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__beat_name, ::Ice::Idempotent, __context);
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::setID(const ::std::string& id, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__setID_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

::std::string
IceDelegateM::cast::interfaces::CASTComponent::getID(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__getID_name, ::Ice::Idempotent, __context);
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::std::string __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::configure(const ::cast::cdl::StringMap& config, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__configure_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::cast::cdl::__writeStringMap(__os, config);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::start(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__start_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::run(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__run_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::stop(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__stop_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::setComponentManager(const ::cast::interfaces::ComponentManagerPrx& man, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__setComponentManager_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(man.get())));
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::setTimeServer(const ::cast::interfaces::TimeServerPrx& ts, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__setTimeServer_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(ts.get())));
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::CASTComponent::destroy(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__CASTComponent__destroy_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemoryAttachedComponent::setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemoryAttachedComponent__setWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(wm.get())));
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemoryReaderComponent::receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemoryReaderComponent__receiveChangeEvent_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        wmc.__write(__os);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::ManagedComponent::setTaskManager(const ::cast::interfaces::TaskManagerPrx& tm, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ManagedComponent__setTaskManager_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(tm.get())));
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::ManagedComponent::taskDecision(const ::std::string& id, ::cast::cdl::TaskManagementDecision decision, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ManagedComponent__taskDecision_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        ::cast::cdl::__write(__os, decision);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::TaskManager::proposeTask(const ::std::string& component, const ::std::string& taskID, const ::std::string& taskName, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TaskManager__proposeTask_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(component);
        __os->write(taskID);
        __os->write(taskName);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::TaskManager::retractTask(const ::std::string& component, const ::std::string& taskID, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TaskManager__retractTask_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(component);
        __os->write(taskID);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::TaskManager::taskComplete(const ::std::string& component, const ::std::string& taskID, ::cast::cdl::TaskOutcome outcome, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TaskManager__taskComplete_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(component);
        __os->write(taskID);
        ::cast::cdl::__write(__os, outcome);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::TaskManager::addManagedComponent(const ::cast::interfaces::ManagedComponentPrx& comp, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__TaskManager__addManagedComponent_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(comp.get())));
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

bool
IceDelegateM::cast::interfaces::WorkingMemory::exists(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__exists_name, ::Ice::Idempotent, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        bool __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::Ice::Int
IceDelegateM::cast::interfaces::WorkingMemory::getVersionNumber(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__getVersionNumber_name, ::Ice::Idempotent, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::Ice::Int __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::cdl::WorkingMemoryPermissions
IceDelegateM::cast::interfaces::WorkingMemory::getPermissions(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__getPermissions_name, ::Ice::Idempotent, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::cdl::WorkingMemoryPermissions __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::cdl::__read(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::lockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__lockEntry_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(component);
        ::cast::cdl::__write(__os, permissions);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

bool
IceDelegateM::cast::interfaces::WorkingMemory::tryLockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__tryLockEntry_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(component);
        ::cast::cdl::__write(__os, permissions);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        bool __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::unlockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__unlockEntry_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(component);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::ConsistencyException&)
            {
                throw;
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::std::string& subarch, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__setWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(wm.get())));
        __os->write(subarch);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::addToWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__addToWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(type);
        __os->write(component);
        __os->write(entry);
        __os->writePendingObjects();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::AlreadyExistsOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::overwriteWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__overwriteWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(type);
        __os->write(component);
        __os->write(entry);
        __os->writePendingObjects();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::deleteFromWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__deleteFromWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(component);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::cdl::WorkingMemoryEntryPtr
IceDelegateM::cast::interfaces::WorkingMemory::getWorkingMemoryEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__getWorkingMemoryEntry_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(subarch);
        __os->write(component);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::DoesNotExistOnWMException&)
            {
                throw;
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::cdl::WorkingMemoryEntryPtr __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(::cast::cdl::__patch__WorkingMemoryEntryPtr, &__ret);
        __is->readPendingObjects();
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::getWorkingMemoryEntries(const ::std::string& type, const ::std::string& subarch, ::Ice::Int count, const ::std::string& component, ::cast::cdl::WorkingMemoryEntrySeq& entries, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__getWorkingMemoryEntries_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(type);
        __os->write(subarch);
        __os->write(count);
        __os->write(component);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::UnknownSubarchitectureException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::cdl::__readWorkingMemoryEntrySeq(__is, entries);
        __is->readPendingObjects();
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__registerComponentFilter_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        filter.__write(__os);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__removeComponentFilter_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        filter.__write(__os);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::std::string& subarch, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__registerWorkingMemoryFilter_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        filter.__write(__os);
        __os->write(subarch);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__removeWorkingMemoryFilter_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        filter.__write(__os);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx& reader, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__addReader_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(reader.get())));
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::cast::interfaces::WorkingMemory::receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__WorkingMemory__receiveChangeEvent_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        wmc.__write(__os);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

::cast::cdl::CASTTime
IceDelegateM::cast::interfaces::ComponentManager::getCASTTime(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ComponentManager__getCASTTime_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::cdl::CASTTime __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __ret.__read(__is);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::interfaces::CASTComponentPrx
IceDelegateM::cast::interfaces::ComponentFactory::newComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ComponentFactory__newComponent_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(type);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::ComponentCreationException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::interfaces::CASTComponentPrx __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::interfaces::__read(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::interfaces::ManagedComponentPrx
IceDelegateM::cast::interfaces::ComponentFactory::newManagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ComponentFactory__newManagedComponent_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(type);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::ComponentCreationException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::interfaces::ManagedComponentPrx __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::interfaces::__read(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::interfaces::UnmanagedComponentPrx
IceDelegateM::cast::interfaces::ComponentFactory::newUnmanagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ComponentFactory__newUnmanagedComponent_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(type);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::ComponentCreationException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::interfaces::UnmanagedComponentPrx __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::interfaces::__read(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::interfaces::WorkingMemoryPrx
IceDelegateM::cast::interfaces::ComponentFactory::newWorkingMemory(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ComponentFactory__newWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(type);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::ComponentCreationException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::interfaces::WorkingMemoryPrx __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::interfaces::__read(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::interfaces::TaskManagerPrx
IceDelegateM::cast::interfaces::ComponentFactory::newTaskManager(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __cast__interfaces__ComponentFactory__newTaskManager_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(id);
        __os->write(type);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::cast::ComponentCreationException&)
            {
                throw;
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::cast::interfaces::TaskManagerPrx __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::cast::interfaces::__read(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::cast::cdl::CASTTime
IceDelegateD::cast::interfaces::TimeServer::getCASTTime(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::cdl::CASTTime& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TimeServer* servant = dynamic_cast< ::cast::interfaces::TimeServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getCASTTime(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::cast::cdl::CASTTime& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TimeServer__getCASTTime_name, ::Ice::Normal, __context);
    ::cast::cdl::CASTTime __result;
    try
    {
        _DirectI __direct(__result, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::cdl::CASTTime
IceDelegateD::cast::interfaces::TimeServer::fromTimeOfDayDouble(::Ice::Double todsecs, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::cdl::CASTTime& __result, ::Ice::Double todsecs, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_todsecs(todsecs)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TimeServer* servant = dynamic_cast< ::cast::interfaces::TimeServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->fromTimeOfDayDouble(_m_todsecs, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::cast::cdl::CASTTime& _result;
        ::Ice::Double _m_todsecs;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TimeServer__fromTimeOfDayDouble_name, ::Ice::Normal, __context);
    ::cast::cdl::CASTTime __result;
    try
    {
        _DirectI __direct(__result, todsecs, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::cdl::CASTTime
IceDelegateD::cast::interfaces::TimeServer::fromTimeOfDay(::Ice::Long secs, ::Ice::Long usecs, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::cdl::CASTTime& __result, ::Ice::Long secs, ::Ice::Long usecs, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_secs(secs),
            _m_usecs(usecs)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TimeServer* servant = dynamic_cast< ::cast::interfaces::TimeServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->fromTimeOfDay(_m_secs, _m_usecs, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::cast::cdl::CASTTime& _result;
        ::Ice::Long _m_secs;
        ::Ice::Long _m_usecs;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TimeServer__fromTimeOfDay_name, ::Ice::Normal, __context);
    ::cast::cdl::CASTTime __result;
    try
    {
        _DirectI __direct(__result, secs, usecs, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

void
IceDelegateD::cast::interfaces::TimeServer::reset(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TimeServer* servant = dynamic_cast< ::cast::interfaces::TimeServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->reset(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TimeServer__reset_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::beat(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->beat(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__beat_name, ::Ice::Idempotent, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::setID(const ::std::string& id, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setID(_m_id, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_id;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__setID_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::std::string
IceDelegateD::cast::interfaces::CASTComponent::getID(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::std::string& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getID(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::std::string& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__getID_name, ::Ice::Idempotent, __context);
    ::std::string __result;
    try
    {
        _DirectI __direct(__result, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

void
IceDelegateD::cast::interfaces::CASTComponent::configure(const ::cast::cdl::StringMap& config, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::StringMap& config, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_config(config)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->configure(_m_config, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::StringMap& _m_config;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__configure_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(config, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::start(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->start(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__start_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::run(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->run(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__run_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::stop(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->stop(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__stop_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::setComponentManager(const ::cast::interfaces::ComponentManagerPrx& man, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::ComponentManagerPrx& man, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_man(man)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setComponentManager(_m_man, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::ComponentManagerPrx& _m_man;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__setComponentManager_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(man, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::setTimeServer(const ::cast::interfaces::TimeServerPrx& ts, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::TimeServerPrx& ts, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_ts(ts)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setTimeServer(_m_ts, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::TimeServerPrx& _m_ts;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__setTimeServer_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(ts, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::CASTComponent::destroy(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::CASTComponent* servant = dynamic_cast< ::cast::interfaces::CASTComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->destroy(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__CASTComponent__destroy_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemoryAttachedComponent::setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_wm(wm)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemoryAttachedComponent* servant = dynamic_cast< ::cast::interfaces::WorkingMemoryAttachedComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setWorkingMemory(_m_wm, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::WorkingMemoryPrx& _m_wm;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemoryAttachedComponent__setWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(wm, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemoryReaderComponent::receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_wmc(wmc)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemoryReaderComponent* servant = dynamic_cast< ::cast::interfaces::WorkingMemoryReaderComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->receiveChangeEvent(_m_wmc, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::WorkingMemoryChange& _m_wmc;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemoryReaderComponent__receiveChangeEvent_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(wmc, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::ManagedComponent::setTaskManager(const ::cast::interfaces::TaskManagerPrx& tm, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::TaskManagerPrx& tm, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_tm(tm)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ManagedComponent* servant = dynamic_cast< ::cast::interfaces::ManagedComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setTaskManager(_m_tm, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::TaskManagerPrx& _m_tm;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ManagedComponent__setTaskManager_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(tm, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::ManagedComponent::taskDecision(const ::std::string& id, ::cast::cdl::TaskManagementDecision decision, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, ::cast::cdl::TaskManagementDecision decision, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id),
            _m_decision(decision)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ManagedComponent* servant = dynamic_cast< ::cast::interfaces::ManagedComponent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->taskDecision(_m_id, _m_decision, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_id;
        ::cast::cdl::TaskManagementDecision _m_decision;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ManagedComponent__taskDecision_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, decision, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::TaskManager::proposeTask(const ::std::string& component, const ::std::string& taskID, const ::std::string& taskName, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& component, const ::std::string& taskID, const ::std::string& taskName, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_component(component),
            _m_taskID(taskID),
            _m_taskName(taskName)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TaskManager* servant = dynamic_cast< ::cast::interfaces::TaskManager*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->proposeTask(_m_component, _m_taskID, _m_taskName, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_component;
        const ::std::string& _m_taskID;
        const ::std::string& _m_taskName;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TaskManager__proposeTask_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(component, taskID, taskName, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::TaskManager::retractTask(const ::std::string& component, const ::std::string& taskID, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& component, const ::std::string& taskID, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_component(component),
            _m_taskID(taskID)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TaskManager* servant = dynamic_cast< ::cast::interfaces::TaskManager*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->retractTask(_m_component, _m_taskID, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_component;
        const ::std::string& _m_taskID;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TaskManager__retractTask_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(component, taskID, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::TaskManager::taskComplete(const ::std::string& component, const ::std::string& taskID, ::cast::cdl::TaskOutcome outcome, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& component, const ::std::string& taskID, ::cast::cdl::TaskOutcome outcome, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_component(component),
            _m_taskID(taskID),
            _m_outcome(outcome)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TaskManager* servant = dynamic_cast< ::cast::interfaces::TaskManager*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->taskComplete(_m_component, _m_taskID, _m_outcome, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_component;
        const ::std::string& _m_taskID;
        ::cast::cdl::TaskOutcome _m_outcome;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TaskManager__taskComplete_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(component, taskID, outcome, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::TaskManager::addManagedComponent(const ::cast::interfaces::ManagedComponentPrx& comp, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::ManagedComponentPrx& comp, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_comp(comp)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::TaskManager* servant = dynamic_cast< ::cast::interfaces::TaskManager*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->addManagedComponent(_m_comp, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::ManagedComponentPrx& _m_comp;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__TaskManager__addManagedComponent_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(comp, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

bool
IceDelegateD::cast::interfaces::WorkingMemory::exists(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(bool& __result, const ::std::string& id, const ::std::string& subarch, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_subarch(subarch)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->exists(_m_id, _m_subarch, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        bool& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__exists_name, ::Ice::Idempotent, __context);
    bool __result;
    try
    {
        _DirectI __direct(__result, id, subarch, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::Ice::Int
IceDelegateD::cast::interfaces::WorkingMemory::getVersionNumber(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int& __result, const ::std::string& id, const ::std::string& subarch, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_subarch(subarch)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->getVersionNumber(_m_id, _m_subarch, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::Ice::Int& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__getVersionNumber_name, ::Ice::Idempotent, __context);
    ::Ice::Int __result;
    try
    {
        _DirectI __direct(__result, id, subarch, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::cdl::WorkingMemoryPermissions
IceDelegateD::cast::interfaces::WorkingMemory::getPermissions(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::cdl::WorkingMemoryPermissions& __result, const ::std::string& id, const ::std::string& subarch, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_subarch(subarch)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->getPermissions(_m_id, _m_subarch, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::cdl::WorkingMemoryPermissions& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__getPermissions_name, ::Ice::Idempotent, __context);
    ::cast::cdl::WorkingMemoryPermissions __result;
    try
    {
        _DirectI __direct(__result, id, subarch, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

void
IceDelegateD::cast::interfaces::WorkingMemory::lockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id),
            _m_subarch(subarch),
            _m_component(component),
            _m_permissions(permissions)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->lockEntry(_m_id, _m_subarch, _m_component, _m_permissions, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_component;
        ::cast::cdl::WorkingMemoryPermissions _m_permissions;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__lockEntry_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, subarch, component, permissions, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

bool
IceDelegateD::cast::interfaces::WorkingMemory::tryLockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(bool& __result, const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_subarch(subarch),
            _m_component(component),
            _m_permissions(permissions)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->tryLockEntry(_m_id, _m_subarch, _m_component, _m_permissions, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        bool& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_component;
        ::cast::cdl::WorkingMemoryPermissions _m_permissions;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__tryLockEntry_name, ::Ice::Normal, __context);
    bool __result;
    try
    {
        _DirectI __direct(__result, id, subarch, component, permissions, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

void
IceDelegateD::cast::interfaces::WorkingMemory::unlockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id),
            _m_subarch(subarch),
            _m_component(component)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->unlockEntry(_m_id, _m_subarch, _m_component, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_component;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__unlockEntry_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, subarch, component, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::ConsistencyException&)
    {
        throw;
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::std::string& subarch, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::std::string& subarch, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_wm(wm),
            _m_subarch(subarch)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setWorkingMemory(_m_wm, _m_subarch, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::WorkingMemoryPrx& _m_wm;
        const ::std::string& _m_subarch;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__setWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(wm, subarch, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::addToWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id),
            _m_subarch(subarch),
            _m_type(type),
            _m_component(component),
            _m_entry(entry)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->addToWorkingMemory(_m_id, _m_subarch, _m_type, _m_component, _m_entry, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_type;
        const ::std::string& _m_component;
        const ::Ice::ObjectPtr& _m_entry;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__addToWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, subarch, type, component, entry, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::AlreadyExistsOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::overwriteWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id),
            _m_subarch(subarch),
            _m_type(type),
            _m_component(component),
            _m_entry(entry)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->overwriteWorkingMemory(_m_id, _m_subarch, _m_type, _m_component, _m_entry, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_type;
        const ::std::string& _m_component;
        const ::Ice::ObjectPtr& _m_entry;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__overwriteWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, subarch, type, component, entry, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::deleteFromWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_id(id),
            _m_subarch(subarch),
            _m_component(component)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->deleteFromWorkingMemory(_m_id, _m_subarch, _m_component, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_component;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__deleteFromWorkingMemory_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(id, subarch, component, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::cast::cdl::WorkingMemoryEntryPtr
IceDelegateD::cast::interfaces::WorkingMemory::getWorkingMemoryEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::cdl::WorkingMemoryEntryPtr& __result, const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_subarch(subarch),
            _m_component(component)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->getWorkingMemoryEntry(_m_id, _m_subarch, _m_component, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::cdl::WorkingMemoryEntryPtr& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_subarch;
        const ::std::string& _m_component;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__getWorkingMemoryEntry_name, ::Ice::Normal, __context);
    ::cast::cdl::WorkingMemoryEntryPtr __result;
    try
    {
        _DirectI __direct(__result, id, subarch, component, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::DoesNotExistOnWMException&)
    {
        throw;
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

void
IceDelegateD::cast::interfaces::WorkingMemory::getWorkingMemoryEntries(const ::std::string& type, const ::std::string& subarch, ::Ice::Int count, const ::std::string& component, ::cast::cdl::WorkingMemoryEntrySeq& entries, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& type, const ::std::string& subarch, ::Ice::Int count, const ::std::string& component, ::cast::cdl::WorkingMemoryEntrySeq& entries, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_type(type),
            _m_subarch(subarch),
            _m_count(count),
            _m_component(component),
            _m_entries(entries)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->getWorkingMemoryEntries(_m_type, _m_subarch, _m_count, _m_component, _m_entries, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_type;
        const ::std::string& _m_subarch;
        ::Ice::Int _m_count;
        const ::std::string& _m_component;
        ::cast::cdl::WorkingMemoryEntrySeq& _m_entries;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__getWorkingMemoryEntries_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(type, subarch, count, component, entries, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::UnknownSubarchitectureException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_filter(filter)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->registerComponentFilter(_m_filter, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::WorkingMemoryChangeFilter& _m_filter;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__registerComponentFilter_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(filter, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_filter(filter)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->removeComponentFilter(_m_filter, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::WorkingMemoryChangeFilter& _m_filter;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__removeComponentFilter_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(filter, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::std::string& subarch, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::std::string& subarch, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_filter(filter),
            _m_subarch(subarch)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->registerWorkingMemoryFilter(_m_filter, _m_subarch, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::WorkingMemoryChangeFilter& _m_filter;
        const ::std::string& _m_subarch;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__registerWorkingMemoryFilter_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(filter, subarch, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_filter(filter)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->removeWorkingMemoryFilter(_m_filter, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::WorkingMemoryChangeFilter& _m_filter;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__removeWorkingMemoryFilter_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(filter, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx& reader, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::interfaces::WorkingMemoryReaderComponentPrx& reader, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_reader(reader)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->addReader(_m_reader, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::interfaces::WorkingMemoryReaderComponentPrx& _m_reader;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__addReader_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(reader, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::cast::interfaces::WorkingMemory::receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_wmc(wmc)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::WorkingMemory* servant = dynamic_cast< ::cast::interfaces::WorkingMemory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->receiveChangeEvent(_m_wmc, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::cast::cdl::WorkingMemoryChange& _m_wmc;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__WorkingMemory__receiveChangeEvent_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(wmc, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::cast::cdl::CASTTime
IceDelegateD::cast::interfaces::ComponentManager::getCASTTime(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::cdl::CASTTime& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ComponentManager* servant = dynamic_cast< ::cast::interfaces::ComponentManager*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getCASTTime(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::cast::cdl::CASTTime& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ComponentManager__getCASTTime_name, ::Ice::Normal, __context);
    ::cast::cdl::CASTTime __result;
    try
    {
        _DirectI __direct(__result, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::interfaces::CASTComponentPrx
IceDelegateD::cast::interfaces::ComponentFactory::newComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::interfaces::CASTComponentPrx& __result, const ::std::string& id, const ::std::string& type, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_type(type)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ComponentFactory* servant = dynamic_cast< ::cast::interfaces::ComponentFactory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->newComponent(_m_id, _m_type, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::interfaces::CASTComponentPrx& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_type;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ComponentFactory__newComponent_name, ::Ice::Normal, __context);
    ::cast::interfaces::CASTComponentPrx __result;
    try
    {
        _DirectI __direct(__result, id, type, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::ComponentCreationException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::interfaces::ManagedComponentPrx
IceDelegateD::cast::interfaces::ComponentFactory::newManagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::interfaces::ManagedComponentPrx& __result, const ::std::string& id, const ::std::string& type, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_type(type)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ComponentFactory* servant = dynamic_cast< ::cast::interfaces::ComponentFactory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->newManagedComponent(_m_id, _m_type, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::interfaces::ManagedComponentPrx& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_type;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ComponentFactory__newManagedComponent_name, ::Ice::Normal, __context);
    ::cast::interfaces::ManagedComponentPrx __result;
    try
    {
        _DirectI __direct(__result, id, type, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::ComponentCreationException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::interfaces::UnmanagedComponentPrx
IceDelegateD::cast::interfaces::ComponentFactory::newUnmanagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::interfaces::UnmanagedComponentPrx& __result, const ::std::string& id, const ::std::string& type, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_type(type)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ComponentFactory* servant = dynamic_cast< ::cast::interfaces::ComponentFactory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->newUnmanagedComponent(_m_id, _m_type, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::interfaces::UnmanagedComponentPrx& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_type;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ComponentFactory__newUnmanagedComponent_name, ::Ice::Normal, __context);
    ::cast::interfaces::UnmanagedComponentPrx __result;
    try
    {
        _DirectI __direct(__result, id, type, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::ComponentCreationException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::interfaces::WorkingMemoryPrx
IceDelegateD::cast::interfaces::ComponentFactory::newWorkingMemory(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::interfaces::WorkingMemoryPrx& __result, const ::std::string& id, const ::std::string& type, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_type(type)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ComponentFactory* servant = dynamic_cast< ::cast::interfaces::ComponentFactory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->newWorkingMemory(_m_id, _m_type, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::interfaces::WorkingMemoryPrx& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_type;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ComponentFactory__newWorkingMemory_name, ::Ice::Normal, __context);
    ::cast::interfaces::WorkingMemoryPrx __result;
    try
    {
        _DirectI __direct(__result, id, type, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::ComponentCreationException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

::cast::interfaces::TaskManagerPrx
IceDelegateD::cast::interfaces::ComponentFactory::newTaskManager(const ::std::string& id, const ::std::string& type, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::cast::interfaces::TaskManagerPrx& __result, const ::std::string& id, const ::std::string& type, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_id(id),
            _m_type(type)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::cast::interfaces::ComponentFactory* servant = dynamic_cast< ::cast::interfaces::ComponentFactory*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                _result = servant->newTaskManager(_m_id, _m_type, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        ::cast::interfaces::TaskManagerPrx& _result;
        const ::std::string& _m_id;
        const ::std::string& _m_type;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __cast__interfaces__ComponentFactory__newTaskManager_name, ::Ice::Normal, __context);
    ::cast::interfaces::TaskManagerPrx __result;
    try
    {
        _DirectI __direct(__result, id, type, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::cast::ComponentCreationException&)
    {
        throw;
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

cast::cdl::TestStructString::TestStructString(const ::std::string& __ice_dummy) :
    dummy(__ice_dummy)
{
}

::Ice::ObjectPtr
cast::cdl::TestStructString::ice_clone() const
{
    ::cast::cdl::TestStructStringPtr __p = new ::cast::cdl::TestStructString(*this);
    return __p;
}

static const ::std::string __cast__cdl__TestStructString_ids[2] =
{
    "::Ice::Object",
    "::cast::cdl::TestStructString"
};

bool
cast::cdl::TestStructString::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__cdl__TestStructString_ids, __cast__cdl__TestStructString_ids + 2, _s);
}

::std::vector< ::std::string>
cast::cdl::TestStructString::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__cdl__TestStructString_ids[0], &__cast__cdl__TestStructString_ids[2]);
}

const ::std::string&
cast::cdl::TestStructString::ice_id(const ::Ice::Current&) const
{
    return __cast__cdl__TestStructString_ids[1];
}

const ::std::string&
cast::cdl::TestStructString::ice_staticId()
{
    return __cast__cdl__TestStructString_ids[1];
}

void
cast::cdl::TestStructString::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(dummy);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::cdl::TestStructString::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(dummy);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::cdl::TestStructString::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::TestStructString was not generated with stream support";
    throw ex;
}

void
cast::cdl::TestStructString::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::TestStructString was not generated with stream support";
    throw ex;
}

class __F__cast__cdl__TestStructString : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::cast::cdl::TestStructString::ice_staticId());
        return new ::cast::cdl::TestStructString;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__cast__cdl__TestStructString_Ptr = new __F__cast__cdl__TestStructString;

const ::Ice::ObjectFactoryPtr&
cast::cdl::TestStructString::ice_factory()
{
    return __F__cast__cdl__TestStructString_Ptr;
}

class __F__cast__cdl__TestStructString__Init
{
public:

    __F__cast__cdl__TestStructString__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::cast::cdl::TestStructString::ice_staticId(), ::cast::cdl::TestStructString::ice_factory());
    }

    ~__F__cast__cdl__TestStructString__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::cast::cdl::TestStructString::ice_staticId());
    }
};

static __F__cast__cdl__TestStructString__Init __F__cast__cdl__TestStructString__i;

#ifdef __APPLE__
extern "C" { void __F__cast__cdl__TestStructString__initializer() {} }
#endif

void 
cast::cdl::__patch__TestStructStringPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::cdl::TestStructStringPtr* p = static_cast< ::cast::cdl::TestStructStringPtr*>(__addr);
    assert(p);
    *p = ::cast::cdl::TestStructStringPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::cdl::TestStructString::ice_staticId(), v->ice_id());
    }
}

bool
cast::cdl::operator==(const ::cast::cdl::TestStructString& l, const ::cast::cdl::TestStructString& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::cdl::operator<(const ::cast::cdl::TestStructString& l, const ::cast::cdl::TestStructString& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

cast::cdl::TestStructInt::TestStructInt(::Ice::Int __ice_dummy) :
    dummy(__ice_dummy)
{
}

::Ice::ObjectPtr
cast::cdl::TestStructInt::ice_clone() const
{
    ::cast::cdl::TestStructIntPtr __p = new ::cast::cdl::TestStructInt(*this);
    return __p;
}

static const ::std::string __cast__cdl__TestStructInt_ids[2] =
{
    "::Ice::Object",
    "::cast::cdl::TestStructInt"
};

bool
cast::cdl::TestStructInt::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__cdl__TestStructInt_ids, __cast__cdl__TestStructInt_ids + 2, _s);
}

::std::vector< ::std::string>
cast::cdl::TestStructInt::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__cdl__TestStructInt_ids[0], &__cast__cdl__TestStructInt_ids[2]);
}

const ::std::string&
cast::cdl::TestStructInt::ice_id(const ::Ice::Current&) const
{
    return __cast__cdl__TestStructInt_ids[1];
}

const ::std::string&
cast::cdl::TestStructInt::ice_staticId()
{
    return __cast__cdl__TestStructInt_ids[1];
}

void
cast::cdl::TestStructInt::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(dummy);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::cdl::TestStructInt::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(dummy);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::cdl::TestStructInt::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::TestStructInt was not generated with stream support";
    throw ex;
}

void
cast::cdl::TestStructInt::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::TestStructInt was not generated with stream support";
    throw ex;
}

class __F__cast__cdl__TestStructInt : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::cast::cdl::TestStructInt::ice_staticId());
        return new ::cast::cdl::TestStructInt;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__cast__cdl__TestStructInt_Ptr = new __F__cast__cdl__TestStructInt;

const ::Ice::ObjectFactoryPtr&
cast::cdl::TestStructInt::ice_factory()
{
    return __F__cast__cdl__TestStructInt_Ptr;
}

class __F__cast__cdl__TestStructInt__Init
{
public:

    __F__cast__cdl__TestStructInt__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::cast::cdl::TestStructInt::ice_staticId(), ::cast::cdl::TestStructInt::ice_factory());
    }

    ~__F__cast__cdl__TestStructInt__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::cast::cdl::TestStructInt::ice_staticId());
    }
};

static __F__cast__cdl__TestStructInt__Init __F__cast__cdl__TestStructInt__i;

#ifdef __APPLE__
extern "C" { void __F__cast__cdl__TestStructInt__initializer() {} }
#endif

void 
cast::cdl::__patch__TestStructIntPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::cdl::TestStructIntPtr* p = static_cast< ::cast::cdl::TestStructIntPtr*>(__addr);
    assert(p);
    *p = ::cast::cdl::TestStructIntPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::cdl::TestStructInt::ice_staticId(), v->ice_id());
    }
}

bool
cast::cdl::operator==(const ::cast::cdl::TestStructInt& l, const ::cast::cdl::TestStructInt& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::cdl::operator<(const ::cast::cdl::TestStructInt& l, const ::cast::cdl::TestStructInt& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

cast::cdl::WorkingMemoryEntry::WorkingMemoryEntry(const ::std::string& __ice_id, const ::std::string& __ice_type, ::Ice::Int __ice_version, const ::Ice::ObjectPtr& __ice_entry) :
    id(__ice_id),
    type(__ice_type),
    version(__ice_version),
    entry(__ice_entry)
{
}

::Ice::ObjectPtr
cast::cdl::WorkingMemoryEntry::ice_clone() const
{
    ::cast::cdl::WorkingMemoryEntryPtr __p = new ::cast::cdl::WorkingMemoryEntry(*this);
    return __p;
}

static const ::std::string __cast__cdl__WorkingMemoryEntry_ids[2] =
{
    "::Ice::Object",
    "::cast::cdl::WorkingMemoryEntry"
};

bool
cast::cdl::WorkingMemoryEntry::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__cdl__WorkingMemoryEntry_ids, __cast__cdl__WorkingMemoryEntry_ids + 2, _s);
}

::std::vector< ::std::string>
cast::cdl::WorkingMemoryEntry::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__cdl__WorkingMemoryEntry_ids[0], &__cast__cdl__WorkingMemoryEntry_ids[2]);
}

const ::std::string&
cast::cdl::WorkingMemoryEntry::ice_id(const ::Ice::Current&) const
{
    return __cast__cdl__WorkingMemoryEntry_ids[1];
}

const ::std::string&
cast::cdl::WorkingMemoryEntry::ice_staticId()
{
    return __cast__cdl__WorkingMemoryEntry_ids[1];
}

void
cast::cdl::WorkingMemoryEntry::__incRef()
{
    __gcIncRef();
}

void
cast::cdl::WorkingMemoryEntry::__decRef()
{
    __gcDecRef();
}

void
cast::cdl::WorkingMemoryEntry::__addObject(::IceInternal::GCCountMap& _c)
{
    ::IceInternal::GCCountMap::iterator pos = _c.find(this);
    if(pos == _c.end())
    {
        _c[this] = 1;
    }
    else
    {
        ++pos->second;
    }
}

bool
cast::cdl::WorkingMemoryEntry::__usesClasses()
{
    return true;
}

void
cast::cdl::WorkingMemoryEntry::__gcReachable(::IceInternal::GCCountMap& _c) const
{
    if(entry)
    {
        entry->__addObject(_c);
    }
}

void
cast::cdl::WorkingMemoryEntry::__gcClear()
{
    if(entry)
    {
        if(entry->__usesClasses())
        {
            entry->__decRefUnsafe();
            entry.__clearHandleUnsafe();
        }
        else
        {
            entry = 0;
        }
    }
}

void
cast::cdl::WorkingMemoryEntry::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(id);
    __os->write(type);
    __os->write(version);
    __os->write(entry);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::cdl::WorkingMemoryEntry::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(id);
    __is->read(type);
    __is->read(version);
    __is->read(::Ice::__patch__ObjectPtr, &entry);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::cdl::WorkingMemoryEntry::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::WorkingMemoryEntry was not generated with stream support";
    throw ex;
}

void
cast::cdl::WorkingMemoryEntry::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::WorkingMemoryEntry was not generated with stream support";
    throw ex;
}

class __F__cast__cdl__WorkingMemoryEntry : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::cast::cdl::WorkingMemoryEntry::ice_staticId());
        return new ::cast::cdl::WorkingMemoryEntry;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__cast__cdl__WorkingMemoryEntry_Ptr = new __F__cast__cdl__WorkingMemoryEntry;

const ::Ice::ObjectFactoryPtr&
cast::cdl::WorkingMemoryEntry::ice_factory()
{
    return __F__cast__cdl__WorkingMemoryEntry_Ptr;
}

class __F__cast__cdl__WorkingMemoryEntry__Init
{
public:

    __F__cast__cdl__WorkingMemoryEntry__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::cast::cdl::WorkingMemoryEntry::ice_staticId(), ::cast::cdl::WorkingMemoryEntry::ice_factory());
    }

    ~__F__cast__cdl__WorkingMemoryEntry__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::cast::cdl::WorkingMemoryEntry::ice_staticId());
    }
};

static __F__cast__cdl__WorkingMemoryEntry__Init __F__cast__cdl__WorkingMemoryEntry__i;

#ifdef __APPLE__
extern "C" { void __F__cast__cdl__WorkingMemoryEntry__initializer() {} }
#endif

void 
cast::cdl::__patch__WorkingMemoryEntryPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::cdl::WorkingMemoryEntryPtr* p = static_cast< ::cast::cdl::WorkingMemoryEntryPtr*>(__addr);
    assert(p);
    *p = ::cast::cdl::WorkingMemoryEntryPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::cdl::WorkingMemoryEntry::ice_staticId(), v->ice_id());
    }
}

bool
cast::cdl::operator==(const ::cast::cdl::WorkingMemoryEntry& l, const ::cast::cdl::WorkingMemoryEntry& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::cdl::operator<(const ::cast::cdl::WorkingMemoryEntry& l, const ::cast::cdl::WorkingMemoryEntry& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

cast::cdl::testing::CASTTestStruct::CASTTestStruct(::Ice::Long __ice_count, const ::cast::cdl::WorkingMemoryChange& __ice_change) :
    count(__ice_count),
    change(__ice_change)
{
}

::Ice::ObjectPtr
cast::cdl::testing::CASTTestStruct::ice_clone() const
{
    ::cast::cdl::testing::CASTTestStructPtr __p = new ::cast::cdl::testing::CASTTestStruct(*this);
    return __p;
}

static const ::std::string __cast__cdl__testing__CASTTestStruct_ids[2] =
{
    "::Ice::Object",
    "::cast::cdl::testing::CASTTestStruct"
};

bool
cast::cdl::testing::CASTTestStruct::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__cdl__testing__CASTTestStruct_ids, __cast__cdl__testing__CASTTestStruct_ids + 2, _s);
}

::std::vector< ::std::string>
cast::cdl::testing::CASTTestStruct::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__cdl__testing__CASTTestStruct_ids[0], &__cast__cdl__testing__CASTTestStruct_ids[2]);
}

const ::std::string&
cast::cdl::testing::CASTTestStruct::ice_id(const ::Ice::Current&) const
{
    return __cast__cdl__testing__CASTTestStruct_ids[1];
}

const ::std::string&
cast::cdl::testing::CASTTestStruct::ice_staticId()
{
    return __cast__cdl__testing__CASTTestStruct_ids[1];
}

void
cast::cdl::testing::CASTTestStruct::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(count);
    change.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::cdl::testing::CASTTestStruct::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(count);
    change.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::cdl::testing::CASTTestStruct::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::testing::CASTTestStruct was not generated with stream support";
    throw ex;
}

void
cast::cdl::testing::CASTTestStruct::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::testing::CASTTestStruct was not generated with stream support";
    throw ex;
}

class __F__cast__cdl__testing__CASTTestStruct : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::cast::cdl::testing::CASTTestStruct::ice_staticId());
        return new ::cast::cdl::testing::CASTTestStruct;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__cast__cdl__testing__CASTTestStruct_Ptr = new __F__cast__cdl__testing__CASTTestStruct;

const ::Ice::ObjectFactoryPtr&
cast::cdl::testing::CASTTestStruct::ice_factory()
{
    return __F__cast__cdl__testing__CASTTestStruct_Ptr;
}

class __F__cast__cdl__testing__CASTTestStruct__Init
{
public:

    __F__cast__cdl__testing__CASTTestStruct__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::cast::cdl::testing::CASTTestStruct::ice_staticId(), ::cast::cdl::testing::CASTTestStruct::ice_factory());
    }

    ~__F__cast__cdl__testing__CASTTestStruct__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::cast::cdl::testing::CASTTestStruct::ice_staticId());
    }
};

static __F__cast__cdl__testing__CASTTestStruct__Init __F__cast__cdl__testing__CASTTestStruct__i;

#ifdef __APPLE__
extern "C" { void __F__cast__cdl__testing__CASTTestStruct__initializer() {} }
#endif

void 
cast::cdl::testing::__patch__CASTTestStructPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::cdl::testing::CASTTestStructPtr* p = static_cast< ::cast::cdl::testing::CASTTestStructPtr*>(__addr);
    assert(p);
    *p = ::cast::cdl::testing::CASTTestStructPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::cdl::testing::CASTTestStruct::ice_staticId(), v->ice_id());
    }
}

bool
cast::cdl::testing::operator==(const ::cast::cdl::testing::CASTTestStruct& l, const ::cast::cdl::testing::CASTTestStruct& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::cdl::testing::operator<(const ::cast::cdl::testing::CASTTestStruct& l, const ::cast::cdl::testing::CASTTestStruct& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

cast::cdl::testing::TestDummyStruct::TestDummyStruct(const ::std::string& __ice_dummy) :
    dummy(__ice_dummy)
{
}

::Ice::ObjectPtr
cast::cdl::testing::TestDummyStruct::ice_clone() const
{
    ::cast::cdl::testing::TestDummyStructPtr __p = new ::cast::cdl::testing::TestDummyStruct(*this);
    return __p;
}

static const ::std::string __cast__cdl__testing__TestDummyStruct_ids[2] =
{
    "::Ice::Object",
    "::cast::cdl::testing::TestDummyStruct"
};

bool
cast::cdl::testing::TestDummyStruct::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__cdl__testing__TestDummyStruct_ids, __cast__cdl__testing__TestDummyStruct_ids + 2, _s);
}

::std::vector< ::std::string>
cast::cdl::testing::TestDummyStruct::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__cdl__testing__TestDummyStruct_ids[0], &__cast__cdl__testing__TestDummyStruct_ids[2]);
}

const ::std::string&
cast::cdl::testing::TestDummyStruct::ice_id(const ::Ice::Current&) const
{
    return __cast__cdl__testing__TestDummyStruct_ids[1];
}

const ::std::string&
cast::cdl::testing::TestDummyStruct::ice_staticId()
{
    return __cast__cdl__testing__TestDummyStruct_ids[1];
}

void
cast::cdl::testing::TestDummyStruct::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(dummy);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::cdl::testing::TestDummyStruct::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(dummy);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::cdl::testing::TestDummyStruct::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::testing::TestDummyStruct was not generated with stream support";
    throw ex;
}

void
cast::cdl::testing::TestDummyStruct::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::cdl::testing::TestDummyStruct was not generated with stream support";
    throw ex;
}

class __F__cast__cdl__testing__TestDummyStruct : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::cast::cdl::testing::TestDummyStruct::ice_staticId());
        return new ::cast::cdl::testing::TestDummyStruct;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__cast__cdl__testing__TestDummyStruct_Ptr = new __F__cast__cdl__testing__TestDummyStruct;

const ::Ice::ObjectFactoryPtr&
cast::cdl::testing::TestDummyStruct::ice_factory()
{
    return __F__cast__cdl__testing__TestDummyStruct_Ptr;
}

class __F__cast__cdl__testing__TestDummyStruct__Init
{
public:

    __F__cast__cdl__testing__TestDummyStruct__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::cast::cdl::testing::TestDummyStruct::ice_staticId(), ::cast::cdl::testing::TestDummyStruct::ice_factory());
    }

    ~__F__cast__cdl__testing__TestDummyStruct__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::cast::cdl::testing::TestDummyStruct::ice_staticId());
    }
};

static __F__cast__cdl__testing__TestDummyStruct__Init __F__cast__cdl__testing__TestDummyStruct__i;

#ifdef __APPLE__
extern "C" { void __F__cast__cdl__testing__TestDummyStruct__initializer() {} }
#endif

void 
cast::cdl::testing::__patch__TestDummyStructPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::cdl::testing::TestDummyStructPtr* p = static_cast< ::cast::cdl::testing::TestDummyStructPtr*>(__addr);
    assert(p);
    *p = ::cast::cdl::testing::TestDummyStructPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::cdl::testing::TestDummyStruct::ice_staticId(), v->ice_id());
    }
}

bool
cast::cdl::testing::operator==(const ::cast::cdl::testing::TestDummyStruct& l, const ::cast::cdl::testing::TestDummyStruct& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::cdl::testing::operator<(const ::cast::cdl::testing::TestDummyStruct& l, const ::cast::cdl::testing::TestDummyStruct& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::TimeServer::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__TimeServer_ids[2] =
{
    "::Ice::Object",
    "::cast::interfaces::TimeServer"
};

bool
cast::interfaces::TimeServer::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__TimeServer_ids, __cast__interfaces__TimeServer_ids + 2, _s);
}

::std::vector< ::std::string>
cast::interfaces::TimeServer::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__TimeServer_ids[0], &__cast__interfaces__TimeServer_ids[2]);
}

const ::std::string&
cast::interfaces::TimeServer::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__TimeServer_ids[1];
}

const ::std::string&
cast::interfaces::TimeServer::ice_staticId()
{
    return __cast__interfaces__TimeServer_ids[1];
}

::Ice::DispatchStatus
cast::interfaces::TimeServer::___getCASTTime(::IceInternal::Incoming& __inS, const ::Ice::Current& __current) const
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::cast::cdl::CASTTime __ret = getCASTTime(__current);
    __ret.__write(__os);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::TimeServer::___fromTimeOfDayDouble(::IceInternal::Incoming& __inS, const ::Ice::Current& __current) const
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Ice::Double todsecs;
    __is->read(todsecs);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::cast::cdl::CASTTime __ret = fromTimeOfDayDouble(todsecs, __current);
    __ret.__write(__os);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::TimeServer::___fromTimeOfDay(::IceInternal::Incoming& __inS, const ::Ice::Current& __current) const
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Ice::Long secs;
    ::Ice::Long usecs;
    __is->read(secs);
    __is->read(usecs);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::cast::cdl::CASTTime __ret = fromTimeOfDay(secs, usecs, __current);
    __ret.__write(__os);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::TimeServer::___reset(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    reset(__current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__TimeServer_all[] =
{
    "fromTimeOfDay",
    "fromTimeOfDayDouble",
    "getCASTTime",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "reset"
};

::Ice::DispatchStatus
cast::interfaces::TimeServer::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__TimeServer_all, __cast__interfaces__TimeServer_all + 8, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__TimeServer_all)
    {
        case 0:
        {
            return ___fromTimeOfDay(in, current);
        }
        case 1:
        {
            return ___fromTimeOfDayDouble(in, current);
        }
        case 2:
        {
            return ___getCASTTime(in, current);
        }
        case 3:
        {
            return ___ice_id(in, current);
        }
        case 4:
        {
            return ___ice_ids(in, current);
        }
        case 5:
        {
            return ___ice_isA(in, current);
        }
        case 6:
        {
            return ___ice_ping(in, current);
        }
        case 7:
        {
            return ___reset(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::TimeServer::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::TimeServer::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::TimeServer::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::TimeServer was not generated with stream support";
    throw ex;
}

void
cast::interfaces::TimeServer::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::TimeServer was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__TimeServerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::TimeServerPtr* p = static_cast< ::cast::interfaces::TimeServerPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::TimeServerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::TimeServer::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::TimeServer& l, const ::cast::interfaces::TimeServer& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::TimeServer& l, const ::cast::interfaces::TimeServer& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::CASTComponent::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__CASTComponent_ids[2] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent"
};

bool
cast::interfaces::CASTComponent::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__CASTComponent_ids, __cast__interfaces__CASTComponent_ids + 2, _s);
}

::std::vector< ::std::string>
cast::interfaces::CASTComponent::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__CASTComponent_ids[0], &__cast__interfaces__CASTComponent_ids[2]);
}

const ::std::string&
cast::interfaces::CASTComponent::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__CASTComponent_ids[1];
}

const ::std::string&
cast::interfaces::CASTComponent::ice_staticId()
{
    return __cast__interfaces__CASTComponent_ids[1];
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___beat(::IceInternal::Incoming& __inS, const ::Ice::Current& __current) const
{
    __checkMode(::Ice::Idempotent, __current.mode);
    __inS.is()->skipEmptyEncaps();
    beat(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___setID(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    __is->read(id);
    __is->endReadEncaps();
    setID(id, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___getID(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Idempotent, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::std::string __ret = getID(__current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___configure(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::StringMap config;
    ::cast::cdl::__readStringMap(__is, config);
    __is->endReadEncaps();
    configure(config, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___start(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    start(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___run(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    run(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___stop(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    stop(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___setComponentManager(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::ComponentManagerPrx man;
    ::cast::interfaces::__read(__is, man);
    __is->endReadEncaps();
    setComponentManager(man, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___setTimeServer(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::TimeServerPrx ts;
    ::cast::interfaces::__read(__is, ts);
    __is->endReadEncaps();
    setTimeServer(ts, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::CASTComponent::___destroy(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    destroy(__current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__CASTComponent_all[] =
{
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "start",
    "stop"
};

::Ice::DispatchStatus
cast::interfaces::CASTComponent::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__CASTComponent_all, __cast__interfaces__CASTComponent_all + 14, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__CASTComponent_all)
    {
        case 0:
        {
            return ___beat(in, current);
        }
        case 1:
        {
            return ___configure(in, current);
        }
        case 2:
        {
            return ___destroy(in, current);
        }
        case 3:
        {
            return ___getID(in, current);
        }
        case 4:
        {
            return ___ice_id(in, current);
        }
        case 5:
        {
            return ___ice_ids(in, current);
        }
        case 6:
        {
            return ___ice_isA(in, current);
        }
        case 7:
        {
            return ___ice_ping(in, current);
        }
        case 8:
        {
            return ___run(in, current);
        }
        case 9:
        {
            return ___setComponentManager(in, current);
        }
        case 10:
        {
            return ___setID(in, current);
        }
        case 11:
        {
            return ___setTimeServer(in, current);
        }
        case 12:
        {
            return ___start(in, current);
        }
        case 13:
        {
            return ___stop(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::CASTComponent::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::CASTComponent::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::CASTComponent::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::CASTComponent was not generated with stream support";
    throw ex;
}

void
cast::interfaces::CASTComponent::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::CASTComponent was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__CASTComponentPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::CASTComponentPtr* p = static_cast< ::cast::interfaces::CASTComponentPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::CASTComponentPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::CASTComponent::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::CASTComponent& l, const ::cast::interfaces::CASTComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::CASTComponent& l, const ::cast::interfaces::CASTComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::WorkingMemoryAttachedComponent::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__WorkingMemoryAttachedComponent_ids[3] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent",
    "::cast::interfaces::WorkingMemoryAttachedComponent"
};

bool
cast::interfaces::WorkingMemoryAttachedComponent::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__WorkingMemoryAttachedComponent_ids, __cast__interfaces__WorkingMemoryAttachedComponent_ids + 3, _s);
}

::std::vector< ::std::string>
cast::interfaces::WorkingMemoryAttachedComponent::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__WorkingMemoryAttachedComponent_ids[0], &__cast__interfaces__WorkingMemoryAttachedComponent_ids[3]);
}

const ::std::string&
cast::interfaces::WorkingMemoryAttachedComponent::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__WorkingMemoryAttachedComponent_ids[2];
}

const ::std::string&
cast::interfaces::WorkingMemoryAttachedComponent::ice_staticId()
{
    return __cast__interfaces__WorkingMemoryAttachedComponent_ids[2];
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemoryAttachedComponent::___setWorkingMemory(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::WorkingMemoryPrx wm;
    ::cast::interfaces::__read(__is, wm);
    __is->endReadEncaps();
    setWorkingMemory(wm, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__WorkingMemoryAttachedComponent_all[] =
{
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "setWorkingMemory",
    "start",
    "stop"
};

::Ice::DispatchStatus
cast::interfaces::WorkingMemoryAttachedComponent::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__WorkingMemoryAttachedComponent_all, __cast__interfaces__WorkingMemoryAttachedComponent_all + 15, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__WorkingMemoryAttachedComponent_all)
    {
        case 0:
        {
            return ___beat(in, current);
        }
        case 1:
        {
            return ___configure(in, current);
        }
        case 2:
        {
            return ___destroy(in, current);
        }
        case 3:
        {
            return ___getID(in, current);
        }
        case 4:
        {
            return ___ice_id(in, current);
        }
        case 5:
        {
            return ___ice_ids(in, current);
        }
        case 6:
        {
            return ___ice_isA(in, current);
        }
        case 7:
        {
            return ___ice_ping(in, current);
        }
        case 8:
        {
            return ___run(in, current);
        }
        case 9:
        {
            return ___setComponentManager(in, current);
        }
        case 10:
        {
            return ___setID(in, current);
        }
        case 11:
        {
            return ___setTimeServer(in, current);
        }
        case 12:
        {
            return ___setWorkingMemory(in, current);
        }
        case 13:
        {
            return ___start(in, current);
        }
        case 14:
        {
            return ___stop(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::WorkingMemoryAttachedComponent::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::WorkingMemoryAttachedComponent::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::WorkingMemoryAttachedComponent::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::WorkingMemoryAttachedComponent was not generated with stream support";
    throw ex;
}

void
cast::interfaces::WorkingMemoryAttachedComponent::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::WorkingMemoryAttachedComponent was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__WorkingMemoryAttachedComponentPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::WorkingMemoryAttachedComponentPtr* p = static_cast< ::cast::interfaces::WorkingMemoryAttachedComponentPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::WorkingMemoryAttachedComponentPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::WorkingMemoryAttachedComponent::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::WorkingMemoryAttachedComponent& l, const ::cast::interfaces::WorkingMemoryAttachedComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::WorkingMemoryAttachedComponent& l, const ::cast::interfaces::WorkingMemoryAttachedComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::WorkingMemoryReaderComponent::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__WorkingMemoryReaderComponent_ids[4] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent",
    "::cast::interfaces::WorkingMemoryAttachedComponent",
    "::cast::interfaces::WorkingMemoryReaderComponent"
};

bool
cast::interfaces::WorkingMemoryReaderComponent::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__WorkingMemoryReaderComponent_ids, __cast__interfaces__WorkingMemoryReaderComponent_ids + 4, _s);
}

::std::vector< ::std::string>
cast::interfaces::WorkingMemoryReaderComponent::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__WorkingMemoryReaderComponent_ids[0], &__cast__interfaces__WorkingMemoryReaderComponent_ids[4]);
}

const ::std::string&
cast::interfaces::WorkingMemoryReaderComponent::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__WorkingMemoryReaderComponent_ids[3];
}

const ::std::string&
cast::interfaces::WorkingMemoryReaderComponent::ice_staticId()
{
    return __cast__interfaces__WorkingMemoryReaderComponent_ids[3];
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemoryReaderComponent::___receiveChangeEvent(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::WorkingMemoryChange wmc;
    wmc.__read(__is);
    __is->endReadEncaps();
    receiveChangeEvent(wmc, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__WorkingMemoryReaderComponent_all[] =
{
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "receiveChangeEvent",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "setWorkingMemory",
    "start",
    "stop"
};

::Ice::DispatchStatus
cast::interfaces::WorkingMemoryReaderComponent::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__WorkingMemoryReaderComponent_all, __cast__interfaces__WorkingMemoryReaderComponent_all + 16, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__WorkingMemoryReaderComponent_all)
    {
        case 0:
        {
            return ___beat(in, current);
        }
        case 1:
        {
            return ___configure(in, current);
        }
        case 2:
        {
            return ___destroy(in, current);
        }
        case 3:
        {
            return ___getID(in, current);
        }
        case 4:
        {
            return ___ice_id(in, current);
        }
        case 5:
        {
            return ___ice_ids(in, current);
        }
        case 6:
        {
            return ___ice_isA(in, current);
        }
        case 7:
        {
            return ___ice_ping(in, current);
        }
        case 8:
        {
            return ___receiveChangeEvent(in, current);
        }
        case 9:
        {
            return ___run(in, current);
        }
        case 10:
        {
            return ___setComponentManager(in, current);
        }
        case 11:
        {
            return ___setID(in, current);
        }
        case 12:
        {
            return ___setTimeServer(in, current);
        }
        case 13:
        {
            return ___setWorkingMemory(in, current);
        }
        case 14:
        {
            return ___start(in, current);
        }
        case 15:
        {
            return ___stop(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::WorkingMemoryReaderComponent::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::WorkingMemoryReaderComponent::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::WorkingMemoryReaderComponent::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::WorkingMemoryReaderComponent was not generated with stream support";
    throw ex;
}

void
cast::interfaces::WorkingMemoryReaderComponent::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::WorkingMemoryReaderComponent was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__WorkingMemoryReaderComponentPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::WorkingMemoryReaderComponentPtr* p = static_cast< ::cast::interfaces::WorkingMemoryReaderComponentPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::WorkingMemoryReaderComponentPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::WorkingMemoryReaderComponent::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::WorkingMemoryReaderComponent& l, const ::cast::interfaces::WorkingMemoryReaderComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::WorkingMemoryReaderComponent& l, const ::cast::interfaces::WorkingMemoryReaderComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::ManagedComponent::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__ManagedComponent_ids[5] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent",
    "::cast::interfaces::ManagedComponent",
    "::cast::interfaces::WorkingMemoryAttachedComponent",
    "::cast::interfaces::WorkingMemoryReaderComponent"
};

bool
cast::interfaces::ManagedComponent::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__ManagedComponent_ids, __cast__interfaces__ManagedComponent_ids + 5, _s);
}

::std::vector< ::std::string>
cast::interfaces::ManagedComponent::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__ManagedComponent_ids[0], &__cast__interfaces__ManagedComponent_ids[5]);
}

const ::std::string&
cast::interfaces::ManagedComponent::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__ManagedComponent_ids[2];
}

const ::std::string&
cast::interfaces::ManagedComponent::ice_staticId()
{
    return __cast__interfaces__ManagedComponent_ids[2];
}

::Ice::DispatchStatus
cast::interfaces::ManagedComponent::___setTaskManager(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::TaskManagerPrx tm;
    ::cast::interfaces::__read(__is, tm);
    __is->endReadEncaps();
    setTaskManager(tm, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::ManagedComponent::___taskDecision(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::cast::cdl::TaskManagementDecision decision;
    __is->read(id);
    ::cast::cdl::__read(__is, decision);
    __is->endReadEncaps();
    taskDecision(id, decision, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__ManagedComponent_all[] =
{
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "receiveChangeEvent",
    "run",
    "setComponentManager",
    "setID",
    "setTaskManager",
    "setTimeServer",
    "setWorkingMemory",
    "start",
    "stop",
    "taskDecision"
};

::Ice::DispatchStatus
cast::interfaces::ManagedComponent::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__ManagedComponent_all, __cast__interfaces__ManagedComponent_all + 18, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__ManagedComponent_all)
    {
        case 0:
        {
            return ___beat(in, current);
        }
        case 1:
        {
            return ___configure(in, current);
        }
        case 2:
        {
            return ___destroy(in, current);
        }
        case 3:
        {
            return ___getID(in, current);
        }
        case 4:
        {
            return ___ice_id(in, current);
        }
        case 5:
        {
            return ___ice_ids(in, current);
        }
        case 6:
        {
            return ___ice_isA(in, current);
        }
        case 7:
        {
            return ___ice_ping(in, current);
        }
        case 8:
        {
            return ___receiveChangeEvent(in, current);
        }
        case 9:
        {
            return ___run(in, current);
        }
        case 10:
        {
            return ___setComponentManager(in, current);
        }
        case 11:
        {
            return ___setID(in, current);
        }
        case 12:
        {
            return ___setTaskManager(in, current);
        }
        case 13:
        {
            return ___setTimeServer(in, current);
        }
        case 14:
        {
            return ___setWorkingMemory(in, current);
        }
        case 15:
        {
            return ___start(in, current);
        }
        case 16:
        {
            return ___stop(in, current);
        }
        case 17:
        {
            return ___taskDecision(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::ManagedComponent::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::ManagedComponent::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::ManagedComponent::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::ManagedComponent was not generated with stream support";
    throw ex;
}

void
cast::interfaces::ManagedComponent::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::ManagedComponent was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__ManagedComponentPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::ManagedComponentPtr* p = static_cast< ::cast::interfaces::ManagedComponentPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::ManagedComponentPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::ManagedComponent::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::ManagedComponent& l, const ::cast::interfaces::ManagedComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::ManagedComponent& l, const ::cast::interfaces::ManagedComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::UnmanagedComponent::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__UnmanagedComponent_ids[4] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent",
    "::cast::interfaces::UnmanagedComponent",
    "::cast::interfaces::WorkingMemoryAttachedComponent"
};

bool
cast::interfaces::UnmanagedComponent::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__UnmanagedComponent_ids, __cast__interfaces__UnmanagedComponent_ids + 4, _s);
}

::std::vector< ::std::string>
cast::interfaces::UnmanagedComponent::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__UnmanagedComponent_ids[0], &__cast__interfaces__UnmanagedComponent_ids[4]);
}

const ::std::string&
cast::interfaces::UnmanagedComponent::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__UnmanagedComponent_ids[2];
}

const ::std::string&
cast::interfaces::UnmanagedComponent::ice_staticId()
{
    return __cast__interfaces__UnmanagedComponent_ids[2];
}

static ::std::string __cast__interfaces__UnmanagedComponent_all[] =
{
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "setWorkingMemory",
    "start",
    "stop"
};

::Ice::DispatchStatus
cast::interfaces::UnmanagedComponent::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__UnmanagedComponent_all, __cast__interfaces__UnmanagedComponent_all + 15, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__UnmanagedComponent_all)
    {
        case 0:
        {
            return ___beat(in, current);
        }
        case 1:
        {
            return ___configure(in, current);
        }
        case 2:
        {
            return ___destroy(in, current);
        }
        case 3:
        {
            return ___getID(in, current);
        }
        case 4:
        {
            return ___ice_id(in, current);
        }
        case 5:
        {
            return ___ice_ids(in, current);
        }
        case 6:
        {
            return ___ice_isA(in, current);
        }
        case 7:
        {
            return ___ice_ping(in, current);
        }
        case 8:
        {
            return ___run(in, current);
        }
        case 9:
        {
            return ___setComponentManager(in, current);
        }
        case 10:
        {
            return ___setID(in, current);
        }
        case 11:
        {
            return ___setTimeServer(in, current);
        }
        case 12:
        {
            return ___setWorkingMemory(in, current);
        }
        case 13:
        {
            return ___start(in, current);
        }
        case 14:
        {
            return ___stop(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::UnmanagedComponent::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::UnmanagedComponent::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::UnmanagedComponent::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::UnmanagedComponent was not generated with stream support";
    throw ex;
}

void
cast::interfaces::UnmanagedComponent::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::UnmanagedComponent was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__UnmanagedComponentPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::UnmanagedComponentPtr* p = static_cast< ::cast::interfaces::UnmanagedComponentPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::UnmanagedComponentPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::UnmanagedComponent::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::UnmanagedComponent& l, const ::cast::interfaces::UnmanagedComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::UnmanagedComponent& l, const ::cast::interfaces::UnmanagedComponent& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::TaskManager::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__TaskManager_ids[5] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent",
    "::cast::interfaces::TaskManager",
    "::cast::interfaces::WorkingMemoryAttachedComponent",
    "::cast::interfaces::WorkingMemoryReaderComponent"
};

bool
cast::interfaces::TaskManager::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__TaskManager_ids, __cast__interfaces__TaskManager_ids + 5, _s);
}

::std::vector< ::std::string>
cast::interfaces::TaskManager::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__TaskManager_ids[0], &__cast__interfaces__TaskManager_ids[5]);
}

const ::std::string&
cast::interfaces::TaskManager::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__TaskManager_ids[2];
}

const ::std::string&
cast::interfaces::TaskManager::ice_staticId()
{
    return __cast__interfaces__TaskManager_ids[2];
}

::Ice::DispatchStatus
cast::interfaces::TaskManager::___proposeTask(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string component;
    ::std::string taskID;
    ::std::string taskName;
    __is->read(component);
    __is->read(taskID);
    __is->read(taskName);
    __is->endReadEncaps();
    proposeTask(component, taskID, taskName, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::TaskManager::___retractTask(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string component;
    ::std::string taskID;
    __is->read(component);
    __is->read(taskID);
    __is->endReadEncaps();
    retractTask(component, taskID, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::TaskManager::___taskComplete(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string component;
    ::std::string taskID;
    ::cast::cdl::TaskOutcome outcome;
    __is->read(component);
    __is->read(taskID);
    ::cast::cdl::__read(__is, outcome);
    __is->endReadEncaps();
    taskComplete(component, taskID, outcome, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::TaskManager::___addManagedComponent(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::ManagedComponentPrx comp;
    ::cast::interfaces::__read(__is, comp);
    __is->endReadEncaps();
    addManagedComponent(comp, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__TaskManager_all[] =
{
    "addManagedComponent",
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "proposeTask",
    "receiveChangeEvent",
    "retractTask",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "setWorkingMemory",
    "start",
    "stop",
    "taskComplete"
};

::Ice::DispatchStatus
cast::interfaces::TaskManager::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__TaskManager_all, __cast__interfaces__TaskManager_all + 20, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__TaskManager_all)
    {
        case 0:
        {
            return ___addManagedComponent(in, current);
        }
        case 1:
        {
            return ___beat(in, current);
        }
        case 2:
        {
            return ___configure(in, current);
        }
        case 3:
        {
            return ___destroy(in, current);
        }
        case 4:
        {
            return ___getID(in, current);
        }
        case 5:
        {
            return ___ice_id(in, current);
        }
        case 6:
        {
            return ___ice_ids(in, current);
        }
        case 7:
        {
            return ___ice_isA(in, current);
        }
        case 8:
        {
            return ___ice_ping(in, current);
        }
        case 9:
        {
            return ___proposeTask(in, current);
        }
        case 10:
        {
            return ___receiveChangeEvent(in, current);
        }
        case 11:
        {
            return ___retractTask(in, current);
        }
        case 12:
        {
            return ___run(in, current);
        }
        case 13:
        {
            return ___setComponentManager(in, current);
        }
        case 14:
        {
            return ___setID(in, current);
        }
        case 15:
        {
            return ___setTimeServer(in, current);
        }
        case 16:
        {
            return ___setWorkingMemory(in, current);
        }
        case 17:
        {
            return ___start(in, current);
        }
        case 18:
        {
            return ___stop(in, current);
        }
        case 19:
        {
            return ___taskComplete(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::TaskManager::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::TaskManager::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::TaskManager::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::TaskManager was not generated with stream support";
    throw ex;
}

void
cast::interfaces::TaskManager::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::TaskManager was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__TaskManagerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::TaskManagerPtr* p = static_cast< ::cast::interfaces::TaskManagerPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::TaskManagerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::TaskManager::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::TaskManager& l, const ::cast::interfaces::TaskManager& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::TaskManager& l, const ::cast::interfaces::TaskManager& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::WorkingMemory::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__WorkingMemory_ids[3] =
{
    "::Ice::Object",
    "::cast::interfaces::CASTComponent",
    "::cast::interfaces::WorkingMemory"
};

bool
cast::interfaces::WorkingMemory::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__WorkingMemory_ids, __cast__interfaces__WorkingMemory_ids + 3, _s);
}

::std::vector< ::std::string>
cast::interfaces::WorkingMemory::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__WorkingMemory_ids[0], &__cast__interfaces__WorkingMemory_ids[3]);
}

const ::std::string&
cast::interfaces::WorkingMemory::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__WorkingMemory_ids[2];
}

const ::std::string&
cast::interfaces::WorkingMemory::ice_staticId()
{
    return __cast__interfaces__WorkingMemory_ids[2];
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___exists(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Idempotent, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    __is->read(id);
    __is->read(subarch);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        bool __ret = exists(id, subarch, __current);
        __os->write(__ret);
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___getVersionNumber(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Idempotent, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    __is->read(id);
    __is->read(subarch);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::Ice::Int __ret = getVersionNumber(id, subarch, __current);
        __os->write(__ret);
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___getPermissions(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Idempotent, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    __is->read(id);
    __is->read(subarch);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::cdl::WorkingMemoryPermissions __ret = getPermissions(id, subarch, __current);
        ::cast::cdl::__write(__os, __ret);
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___lockEntry(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string component;
    ::cast::cdl::WorkingMemoryPermissions permissions;
    __is->read(id);
    __is->read(subarch);
    __is->read(component);
    ::cast::cdl::__read(__is, permissions);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        lockEntry(id, subarch, component, permissions, __current);
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___tryLockEntry(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string component;
    ::cast::cdl::WorkingMemoryPermissions permissions;
    __is->read(id);
    __is->read(subarch);
    __is->read(component);
    ::cast::cdl::__read(__is, permissions);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        bool __ret = tryLockEntry(id, subarch, component, permissions, __current);
        __os->write(__ret);
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___unlockEntry(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string component;
    __is->read(id);
    __is->read(subarch);
    __is->read(component);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        unlockEntry(id, subarch, component, __current);
    }
    catch(const ::cast::ConsistencyException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___setWorkingMemory(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::WorkingMemoryPrx wm;
    ::std::string subarch;
    ::cast::interfaces::__read(__is, wm);
    __is->read(subarch);
    __is->endReadEncaps();
    setWorkingMemory(wm, subarch, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___addToWorkingMemory(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string type;
    ::std::string component;
    ::Ice::ObjectPtr entry;
    __is->read(id);
    __is->read(subarch);
    __is->read(type);
    __is->read(component);
    __is->read(::Ice::__patch__ObjectPtr, &entry);
    __is->readPendingObjects();
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        addToWorkingMemory(id, subarch, type, component, entry, __current);
    }
    catch(const ::cast::AlreadyExistsOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___overwriteWorkingMemory(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string type;
    ::std::string component;
    ::Ice::ObjectPtr entry;
    __is->read(id);
    __is->read(subarch);
    __is->read(type);
    __is->read(component);
    __is->read(::Ice::__patch__ObjectPtr, &entry);
    __is->readPendingObjects();
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        overwriteWorkingMemory(id, subarch, type, component, entry, __current);
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___deleteFromWorkingMemory(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string component;
    __is->read(id);
    __is->read(subarch);
    __is->read(component);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        deleteFromWorkingMemory(id, subarch, component, __current);
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___getWorkingMemoryEntry(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string subarch;
    ::std::string component;
    __is->read(id);
    __is->read(subarch);
    __is->read(component);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::cdl::WorkingMemoryEntryPtr __ret = getWorkingMemoryEntry(id, subarch, component, __current);
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(__ret.get())));
        __os->writePendingObjects();
    }
    catch(const ::cast::DoesNotExistOnWMException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___getWorkingMemoryEntries(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string type;
    ::std::string subarch;
    ::Ice::Int count;
    ::std::string component;
    __is->read(type);
    __is->read(subarch);
    __is->read(count);
    __is->read(component);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::cast::cdl::WorkingMemoryEntrySeq entries;
    try
    {
        getWorkingMemoryEntries(type, subarch, count, component, entries, __current);
        if(entries.size() == 0)
        {
            __os->writeSize(0);
        }
        else
        {
            ::cast::cdl::__writeWorkingMemoryEntrySeq(__os, &entries[0], &entries[0] + entries.size());
        }
        __os->writePendingObjects();
    }
    catch(const ::cast::UnknownSubarchitectureException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___registerComponentFilter(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::WorkingMemoryChangeFilter filter;
    filter.__read(__is);
    __is->endReadEncaps();
    registerComponentFilter(filter, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___removeComponentFilter(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::WorkingMemoryChangeFilter filter;
    filter.__read(__is);
    __is->endReadEncaps();
    removeComponentFilter(filter, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___registerWorkingMemoryFilter(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::WorkingMemoryChangeFilter filter;
    ::std::string subarch;
    filter.__read(__is);
    __is->read(subarch);
    __is->endReadEncaps();
    registerWorkingMemoryFilter(filter, subarch, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___removeWorkingMemoryFilter(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::WorkingMemoryChangeFilter filter;
    filter.__read(__is);
    __is->endReadEncaps();
    removeWorkingMemoryFilter(filter, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___addReader(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::interfaces::WorkingMemoryReaderComponentPrx reader;
    ::cast::interfaces::__read(__is, reader);
    __is->endReadEncaps();
    addReader(reader, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::___receiveChangeEvent(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::cast::cdl::WorkingMemoryChange wmc;
    wmc.__read(__is);
    __is->endReadEncaps();
    receiveChangeEvent(wmc, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__WorkingMemory_all[] =
{
    "addReader",
    "addToWorkingMemory",
    "beat",
    "configure",
    "deleteFromWorkingMemory",
    "destroy",
    "exists",
    "getID",
    "getPermissions",
    "getVersionNumber",
    "getWorkingMemoryEntries",
    "getWorkingMemoryEntry",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "lockEntry",
    "overwriteWorkingMemory",
    "receiveChangeEvent",
    "registerComponentFilter",
    "registerWorkingMemoryFilter",
    "removeComponentFilter",
    "removeWorkingMemoryFilter",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "setWorkingMemory",
    "start",
    "stop",
    "tryLockEntry",
    "unlockEntry"
};

::Ice::DispatchStatus
cast::interfaces::WorkingMemory::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__WorkingMemory_all, __cast__interfaces__WorkingMemory_all + 32, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__WorkingMemory_all)
    {
        case 0:
        {
            return ___addReader(in, current);
        }
        case 1:
        {
            return ___addToWorkingMemory(in, current);
        }
        case 2:
        {
            return ___beat(in, current);
        }
        case 3:
        {
            return ___configure(in, current);
        }
        case 4:
        {
            return ___deleteFromWorkingMemory(in, current);
        }
        case 5:
        {
            return ___destroy(in, current);
        }
        case 6:
        {
            return ___exists(in, current);
        }
        case 7:
        {
            return ___getID(in, current);
        }
        case 8:
        {
            return ___getPermissions(in, current);
        }
        case 9:
        {
            return ___getVersionNumber(in, current);
        }
        case 10:
        {
            return ___getWorkingMemoryEntries(in, current);
        }
        case 11:
        {
            return ___getWorkingMemoryEntry(in, current);
        }
        case 12:
        {
            return ___ice_id(in, current);
        }
        case 13:
        {
            return ___ice_ids(in, current);
        }
        case 14:
        {
            return ___ice_isA(in, current);
        }
        case 15:
        {
            return ___ice_ping(in, current);
        }
        case 16:
        {
            return ___lockEntry(in, current);
        }
        case 17:
        {
            return ___overwriteWorkingMemory(in, current);
        }
        case 18:
        {
            return ___receiveChangeEvent(in, current);
        }
        case 19:
        {
            return ___registerComponentFilter(in, current);
        }
        case 20:
        {
            return ___registerWorkingMemoryFilter(in, current);
        }
        case 21:
        {
            return ___removeComponentFilter(in, current);
        }
        case 22:
        {
            return ___removeWorkingMemoryFilter(in, current);
        }
        case 23:
        {
            return ___run(in, current);
        }
        case 24:
        {
            return ___setComponentManager(in, current);
        }
        case 25:
        {
            return ___setID(in, current);
        }
        case 26:
        {
            return ___setTimeServer(in, current);
        }
        case 27:
        {
            return ___setWorkingMemory(in, current);
        }
        case 28:
        {
            return ___start(in, current);
        }
        case 29:
        {
            return ___stop(in, current);
        }
        case 30:
        {
            return ___tryLockEntry(in, current);
        }
        case 31:
        {
            return ___unlockEntry(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::WorkingMemory::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::WorkingMemory::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::WorkingMemory::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::WorkingMemory was not generated with stream support";
    throw ex;
}

void
cast::interfaces::WorkingMemory::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::WorkingMemory was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__WorkingMemoryPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::WorkingMemoryPtr* p = static_cast< ::cast::interfaces::WorkingMemoryPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::WorkingMemoryPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::WorkingMemory::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::WorkingMemory& l, const ::cast::interfaces::WorkingMemory& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::WorkingMemory& l, const ::cast::interfaces::WorkingMemory& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::ComponentManager::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__ComponentManager_ids[2] =
{
    "::Ice::Object",
    "::cast::interfaces::ComponentManager"
};

bool
cast::interfaces::ComponentManager::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__ComponentManager_ids, __cast__interfaces__ComponentManager_ids + 2, _s);
}

::std::vector< ::std::string>
cast::interfaces::ComponentManager::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__ComponentManager_ids[0], &__cast__interfaces__ComponentManager_ids[2]);
}

const ::std::string&
cast::interfaces::ComponentManager::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__ComponentManager_ids[1];
}

const ::std::string&
cast::interfaces::ComponentManager::ice_staticId()
{
    return __cast__interfaces__ComponentManager_ids[1];
}

::Ice::DispatchStatus
cast::interfaces::ComponentManager::___getCASTTime(::IceInternal::Incoming& __inS, const ::Ice::Current& __current) const
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::cast::cdl::CASTTime __ret = getCASTTime(__current);
    __ret.__write(__os);
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__ComponentManager_all[] =
{
    "getCASTTime",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

::Ice::DispatchStatus
cast::interfaces::ComponentManager::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__ComponentManager_all, __cast__interfaces__ComponentManager_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__ComponentManager_all)
    {
        case 0:
        {
            return ___getCASTTime(in, current);
        }
        case 1:
        {
            return ___ice_id(in, current);
        }
        case 2:
        {
            return ___ice_ids(in, current);
        }
        case 3:
        {
            return ___ice_isA(in, current);
        }
        case 4:
        {
            return ___ice_ping(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::ComponentManager::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::ComponentManager::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::ComponentManager::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::ComponentManager was not generated with stream support";
    throw ex;
}

void
cast::interfaces::ComponentManager::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::ComponentManager was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__ComponentManagerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::ComponentManagerPtr* p = static_cast< ::cast::interfaces::ComponentManagerPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::ComponentManagerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::ComponentManager::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::ComponentManager& l, const ::cast::interfaces::ComponentManager& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::ComponentManager& l, const ::cast::interfaces::ComponentManager& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
cast::interfaces::ComponentFactory::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __cast__interfaces__ComponentFactory_ids[2] =
{
    "::Ice::Object",
    "::cast::interfaces::ComponentFactory"
};

bool
cast::interfaces::ComponentFactory::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__cast__interfaces__ComponentFactory_ids, __cast__interfaces__ComponentFactory_ids + 2, _s);
}

::std::vector< ::std::string>
cast::interfaces::ComponentFactory::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__cast__interfaces__ComponentFactory_ids[0], &__cast__interfaces__ComponentFactory_ids[2]);
}

const ::std::string&
cast::interfaces::ComponentFactory::ice_id(const ::Ice::Current&) const
{
    return __cast__interfaces__ComponentFactory_ids[1];
}

const ::std::string&
cast::interfaces::ComponentFactory::ice_staticId()
{
    return __cast__interfaces__ComponentFactory_ids[1];
}

::Ice::DispatchStatus
cast::interfaces::ComponentFactory::___newComponent(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string type;
    __is->read(id);
    __is->read(type);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::interfaces::CASTComponentPrx __ret = newComponent(id, type, __current);
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(__ret.get())));
    }
    catch(const ::cast::ComponentCreationException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::ComponentFactory::___newManagedComponent(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string type;
    __is->read(id);
    __is->read(type);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::interfaces::ManagedComponentPrx __ret = newManagedComponent(id, type, __current);
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(__ret.get())));
    }
    catch(const ::cast::ComponentCreationException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::ComponentFactory::___newUnmanagedComponent(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string type;
    __is->read(id);
    __is->read(type);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::interfaces::UnmanagedComponentPrx __ret = newUnmanagedComponent(id, type, __current);
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(__ret.get())));
    }
    catch(const ::cast::ComponentCreationException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::ComponentFactory::___newWorkingMemory(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string type;
    __is->read(id);
    __is->read(type);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::interfaces::WorkingMemoryPrx __ret = newWorkingMemory(id, type, __current);
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(__ret.get())));
    }
    catch(const ::cast::ComponentCreationException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
cast::interfaces::ComponentFactory::___newTaskManager(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string id;
    ::std::string type;
    __is->read(id);
    __is->read(type);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        ::cast::interfaces::TaskManagerPrx __ret = newTaskManager(id, type, __current);
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(__ret.get())));
    }
    catch(const ::cast::ComponentCreationException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

static ::std::string __cast__interfaces__ComponentFactory_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "newComponent",
    "newManagedComponent",
    "newTaskManager",
    "newUnmanagedComponent",
    "newWorkingMemory"
};

::Ice::DispatchStatus
cast::interfaces::ComponentFactory::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__cast__interfaces__ComponentFactory_all, __cast__interfaces__ComponentFactory_all + 9, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __cast__interfaces__ComponentFactory_all)
    {
        case 0:
        {
            return ___ice_id(in, current);
        }
        case 1:
        {
            return ___ice_ids(in, current);
        }
        case 2:
        {
            return ___ice_isA(in, current);
        }
        case 3:
        {
            return ___ice_ping(in, current);
        }
        case 4:
        {
            return ___newComponent(in, current);
        }
        case 5:
        {
            return ___newManagedComponent(in, current);
        }
        case 6:
        {
            return ___newTaskManager(in, current);
        }
        case 7:
        {
            return ___newUnmanagedComponent(in, current);
        }
        case 8:
        {
            return ___newWorkingMemory(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
cast::interfaces::ComponentFactory::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
cast::interfaces::ComponentFactory::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
cast::interfaces::ComponentFactory::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::ComponentFactory was not generated with stream support";
    throw ex;
}

void
cast::interfaces::ComponentFactory::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type cast::interfaces::ComponentFactory was not generated with stream support";
    throw ex;
}

void 
cast::interfaces::__patch__ComponentFactoryPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::cast::interfaces::ComponentFactoryPtr* p = static_cast< ::cast::interfaces::ComponentFactoryPtr*>(__addr);
    assert(p);
    *p = ::cast::interfaces::ComponentFactoryPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::cast::interfaces::ComponentFactory::ice_staticId(), v->ice_id());
    }
}

bool
cast::interfaces::operator==(const ::cast::interfaces::ComponentFactory& l, const ::cast::interfaces::ComponentFactory& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
cast::interfaces::operator<(const ::cast::interfaces::ComponentFactory& l, const ::cast::interfaces::ComponentFactory& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
