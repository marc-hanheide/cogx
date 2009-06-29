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

#ifndef ___home_plison_svn_cogx_binding_development_tools_cast_src_c___cast_slice_CDL_hpp__
#define ___home_plison_svn_cogx_binding_development_tools_cast_src_c___cast_slice_CDL_hpp__

#include <Ice/LocalObjectF.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <Ice/UserExceptionFactory.h>
#include <Ice/FactoryTable.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

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

namespace IceProxy
{

namespace cast
{

namespace cdl
{

class TestStructString;

class TestStructInt;

class WorkingMemoryEntry;

namespace testing
{

class CASTTestStruct;

class TestDummyStruct;

}

}

namespace interfaces
{

class WorkingMemory;

class TaskManager;

class ComponentManager;

class TimeServer;

class CASTComponent;

class WorkingMemoryAttachedComponent;

class WorkingMemoryReaderComponent;

class ManagedComponent;

class UnmanagedComponent;

class ComponentFactory;

}

}

}

namespace cast
{

namespace cdl
{

class TestStructString;
bool operator==(const TestStructString&, const TestStructString&);
bool operator<(const TestStructString&, const TestStructString&);

class TestStructInt;
bool operator==(const TestStructInt&, const TestStructInt&);
bool operator<(const TestStructInt&, const TestStructInt&);

class WorkingMemoryEntry;
bool operator==(const WorkingMemoryEntry&, const WorkingMemoryEntry&);
bool operator<(const WorkingMemoryEntry&, const WorkingMemoryEntry&);

namespace testing
{

class CASTTestStruct;
bool operator==(const CASTTestStruct&, const CASTTestStruct&);
bool operator<(const CASTTestStruct&, const CASTTestStruct&);

class TestDummyStruct;
bool operator==(const TestDummyStruct&, const TestDummyStruct&);
bool operator<(const TestDummyStruct&, const TestDummyStruct&);

}

}

namespace interfaces
{

class WorkingMemory;
bool operator==(const WorkingMemory&, const WorkingMemory&);
bool operator<(const WorkingMemory&, const WorkingMemory&);

class TaskManager;
bool operator==(const TaskManager&, const TaskManager&);
bool operator<(const TaskManager&, const TaskManager&);

class ComponentManager;
bool operator==(const ComponentManager&, const ComponentManager&);
bool operator<(const ComponentManager&, const ComponentManager&);

class TimeServer;
bool operator==(const TimeServer&, const TimeServer&);
bool operator<(const TimeServer&, const TimeServer&);

class CASTComponent;
bool operator==(const CASTComponent&, const CASTComponent&);
bool operator<(const CASTComponent&, const CASTComponent&);

class WorkingMemoryAttachedComponent;
bool operator==(const WorkingMemoryAttachedComponent&, const WorkingMemoryAttachedComponent&);
bool operator<(const WorkingMemoryAttachedComponent&, const WorkingMemoryAttachedComponent&);

class WorkingMemoryReaderComponent;
bool operator==(const WorkingMemoryReaderComponent&, const WorkingMemoryReaderComponent&);
bool operator<(const WorkingMemoryReaderComponent&, const WorkingMemoryReaderComponent&);

class ManagedComponent;
bool operator==(const ManagedComponent&, const ManagedComponent&);
bool operator<(const ManagedComponent&, const ManagedComponent&);

class UnmanagedComponent;
bool operator==(const UnmanagedComponent&, const UnmanagedComponent&);
bool operator<(const UnmanagedComponent&, const UnmanagedComponent&);

class ComponentFactory;
bool operator==(const ComponentFactory&, const ComponentFactory&);
bool operator<(const ComponentFactory&, const ComponentFactory&);

}

}

namespace IceInternal
{

::Ice::Object* upCast(::cast::cdl::TestStructString*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::cdl::TestStructString*);

::Ice::Object* upCast(::cast::cdl::TestStructInt*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::cdl::TestStructInt*);

::Ice::Object* upCast(::cast::cdl::WorkingMemoryEntry*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::cdl::WorkingMemoryEntry*);

::Ice::Object* upCast(::cast::cdl::testing::CASTTestStruct*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::cdl::testing::CASTTestStruct*);

::Ice::Object* upCast(::cast::cdl::testing::TestDummyStruct*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::cdl::testing::TestDummyStruct*);

::Ice::Object* upCast(::cast::interfaces::WorkingMemory*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::WorkingMemory*);

::Ice::Object* upCast(::cast::interfaces::TaskManager*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::TaskManager*);

::Ice::Object* upCast(::cast::interfaces::ComponentManager*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::ComponentManager*);

::Ice::Object* upCast(::cast::interfaces::TimeServer*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::TimeServer*);

::Ice::Object* upCast(::cast::interfaces::CASTComponent*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::CASTComponent*);

::Ice::Object* upCast(::cast::interfaces::WorkingMemoryAttachedComponent*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::WorkingMemoryAttachedComponent*);

::Ice::Object* upCast(::cast::interfaces::WorkingMemoryReaderComponent*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::WorkingMemoryReaderComponent*);

::Ice::Object* upCast(::cast::interfaces::ManagedComponent*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::ManagedComponent*);

::Ice::Object* upCast(::cast::interfaces::UnmanagedComponent*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::UnmanagedComponent*);

::Ice::Object* upCast(::cast::interfaces::ComponentFactory*);
::IceProxy::Ice::Object* upCast(::IceProxy::cast::interfaces::ComponentFactory*);

}

namespace cast
{

namespace cdl
{

typedef ::IceInternal::Handle< ::cast::cdl::TestStructString> TestStructStringPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::cdl::TestStructString> TestStructStringPrx;

void __read(::IceInternal::BasicStream*, TestStructStringPrx&);
void __patch__TestStructStringPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::cdl::TestStructInt> TestStructIntPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::cdl::TestStructInt> TestStructIntPrx;

void __read(::IceInternal::BasicStream*, TestStructIntPrx&);
void __patch__TestStructIntPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::cdl::WorkingMemoryEntry> WorkingMemoryEntryPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::cdl::WorkingMemoryEntry> WorkingMemoryEntryPrx;

void __read(::IceInternal::BasicStream*, WorkingMemoryEntryPrx&);
void __patch__WorkingMemoryEntryPtr(void*, ::Ice::ObjectPtr&);

namespace testing
{

typedef ::IceInternal::Handle< ::cast::cdl::testing::CASTTestStruct> CASTTestStructPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::cdl::testing::CASTTestStruct> CASTTestStructPrx;

void __read(::IceInternal::BasicStream*, CASTTestStructPrx&);
void __patch__CASTTestStructPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::cdl::testing::TestDummyStruct> TestDummyStructPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::cdl::testing::TestDummyStruct> TestDummyStructPrx;

void __read(::IceInternal::BasicStream*, TestDummyStructPrx&);
void __patch__TestDummyStructPtr(void*, ::Ice::ObjectPtr&);

}

}

namespace interfaces
{

typedef ::IceInternal::Handle< ::cast::interfaces::WorkingMemory> WorkingMemoryPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::WorkingMemory> WorkingMemoryPrx;

void __read(::IceInternal::BasicStream*, WorkingMemoryPrx&);
void __patch__WorkingMemoryPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::TaskManager> TaskManagerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::TaskManager> TaskManagerPrx;

void __read(::IceInternal::BasicStream*, TaskManagerPrx&);
void __patch__TaskManagerPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::ComponentManager> ComponentManagerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::ComponentManager> ComponentManagerPrx;

void __read(::IceInternal::BasicStream*, ComponentManagerPrx&);
void __patch__ComponentManagerPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::TimeServer> TimeServerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::TimeServer> TimeServerPrx;

void __read(::IceInternal::BasicStream*, TimeServerPrx&);
void __patch__TimeServerPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::CASTComponent> CASTComponentPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::CASTComponent> CASTComponentPrx;

void __read(::IceInternal::BasicStream*, CASTComponentPrx&);
void __patch__CASTComponentPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::WorkingMemoryAttachedComponent> WorkingMemoryAttachedComponentPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::WorkingMemoryAttachedComponent> WorkingMemoryAttachedComponentPrx;

void __read(::IceInternal::BasicStream*, WorkingMemoryAttachedComponentPrx&);
void __patch__WorkingMemoryAttachedComponentPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::WorkingMemoryReaderComponent> WorkingMemoryReaderComponentPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::WorkingMemoryReaderComponent> WorkingMemoryReaderComponentPrx;

void __read(::IceInternal::BasicStream*, WorkingMemoryReaderComponentPrx&);
void __patch__WorkingMemoryReaderComponentPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::ManagedComponent> ManagedComponentPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::ManagedComponent> ManagedComponentPrx;

void __read(::IceInternal::BasicStream*, ManagedComponentPrx&);
void __patch__ManagedComponentPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::UnmanagedComponent> UnmanagedComponentPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::UnmanagedComponent> UnmanagedComponentPrx;

void __read(::IceInternal::BasicStream*, UnmanagedComponentPrx&);
void __patch__UnmanagedComponentPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::cast::interfaces::ComponentFactory> ComponentFactoryPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::cast::interfaces::ComponentFactory> ComponentFactoryPrx;

void __read(::IceInternal::BasicStream*, ComponentFactoryPrx&);
void __patch__ComponentFactoryPtr(void*, ::Ice::ObjectPtr&);

}

}

namespace cast
{

namespace cdl
{

const ::std::string CASTRELEASESTRING = "0.2.0";

const ::Ice::Int JAVASERVERPORT = 10111;

const ::Ice::Int CPPSERVERPORT = 10211;

const ::Ice::Int JAVACLIENTSERVERPORT = 10311;

const ::std::string SUBARCHIDKEY = "org.cognitivesystem.cast.subarchID";

const ::std::string COMPONENTNUMBERKEY = "org.cognitivesystem.cast.componentNumber";

const ::std::string CONFIGFILEKEY = "org.cognitivesystem.cast.config";

const ::std::string WMIDSKEY = "org.cognitivesystem.ast.wmids";

const ::std::string COMPONENTIDSKEY = "org.cognitivesystem.cast.compids";

const ::std::string LOGKEY = "--log";

const ::std::string DEBUGKEY = "--debug";

const ::std::string DEBUGEVENTSKEY = "--debug-events";

const ::std::string IGNORESAKEY = "--ignore";

typedef ::std::map< ::std::string, ::std::string> StringMap;
void __writeStringMap(::IceInternal::BasicStream*, const StringMap&);
void __readStringMap(::IceInternal::BasicStream*, StringMap&);

enum ComponentLanguage
{
    CPP,
    JAVA
};

void __write(::IceInternal::BasicStream*, ComponentLanguage);
void __read(::IceInternal::BasicStream*, ComponentLanguage&);

struct ComponentDescription
{
    ::std::string componentName;
    ::std::string className;
    ::cast::cdl::ComponentLanguage language;
    ::std::string hostName;
    ::cast::cdl::StringMap configuration;

    bool operator==(const ComponentDescription&) const;
    bool operator<(const ComponentDescription&) const;
    bool operator!=(const ComponentDescription& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const ComponentDescription& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const ComponentDescription& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const ComponentDescription& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::Ice::Byte> ByteSeq;

struct WorkingMemoryAddress
{
    ::std::string id;
    ::std::string subarchitecture;

    bool operator==(const WorkingMemoryAddress&) const;
    bool operator<(const WorkingMemoryAddress&) const;
    bool operator!=(const WorkingMemoryAddress& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const WorkingMemoryAddress& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const WorkingMemoryAddress& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const WorkingMemoryAddress& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::cast::cdl::WorkingMemoryEntryPtr> WorkingMemoryEntrySeq;
void __writeWorkingMemoryEntrySeq(::IceInternal::BasicStream*, const ::cast::cdl::WorkingMemoryEntryPtr*, const ::cast::cdl::WorkingMemoryEntryPtr*);
void __readWorkingMemoryEntrySeq(::IceInternal::BasicStream*, WorkingMemoryEntrySeq&);

enum WorkingMemoryOperation
{
    ADD,
    OVERWRITE,
    DELETE,
    GET,
    WILDCARD
};

void __write(::IceInternal::BasicStream*, WorkingMemoryOperation);
void __read(::IceInternal::BasicStream*, WorkingMemoryOperation&);

enum WorkingMemoryChangeQueueBehaviour
{
    DISCARD,
    QUEUE
};

void __write(::IceInternal::BasicStream*, WorkingMemoryChangeQueueBehaviour);
void __read(::IceInternal::BasicStream*, WorkingMemoryChangeQueueBehaviour&);

enum WorkingMemoryPermissions
{
    LOCKEDO,
    LOCKEDOD,
    LOCKEDODR,
    UNLOCKED,
    DOESNOTEXIST,
    ALREADYLOCKED
};

void __write(::IceInternal::BasicStream*, WorkingMemoryPermissions);
void __read(::IceInternal::BasicStream*, WorkingMemoryPermissions&);

enum WorkingMemoryLockRequest
{
    REQUESTLOCKO,
    REQUESTLOCKOD,
    REQUESTLOCKODR,
    REQUESTTRYLOCKO,
    REQUESTTRYLOCKOD,
    REQUESTTRYLOCKODR,
    REQUESTUNLOCK,
    REQUESTSTATUS
};

void __write(::IceInternal::BasicStream*, WorkingMemoryLockRequest);
void __read(::IceInternal::BasicStream*, WorkingMemoryLockRequest&);

enum FilterRestriction
{
    LOCALSA,
    ALLSA
};

void __write(::IceInternal::BasicStream*, FilterRestriction);
void __read(::IceInternal::BasicStream*, FilterRestriction&);

enum ReceiverDeleteCondition
{
    DELETERECEIVER,
    DONOTDELETERECEIVER
};

void __write(::IceInternal::BasicStream*, ReceiverDeleteCondition);
void __read(::IceInternal::BasicStream*, ReceiverDeleteCondition&);

struct WorkingMemoryChange
{
    ::cast::cdl::WorkingMemoryOperation operation;
    ::std::string src;
    ::cast::cdl::WorkingMemoryAddress address;
    ::std::string type;

    bool operator==(const WorkingMemoryChange&) const;
    bool operator<(const WorkingMemoryChange&) const;
    bool operator!=(const WorkingMemoryChange& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const WorkingMemoryChange& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const WorkingMemoryChange& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const WorkingMemoryChange& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct WorkingMemoryChangeFilter
{
    ::cast::cdl::WorkingMemoryOperation operation;
    ::std::string src;
    ::cast::cdl::WorkingMemoryAddress address;
    ::std::string type;
    ::cast::cdl::FilterRestriction restriction;
    ::std::string origin;

    bool operator==(const WorkingMemoryChangeFilter&) const;
    bool operator<(const WorkingMemoryChangeFilter&) const;
    bool operator!=(const WorkingMemoryChangeFilter& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const WorkingMemoryChangeFilter& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const WorkingMemoryChangeFilter& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const WorkingMemoryChangeFilter& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct CASTTime
{
    ::Ice::Long s;
    ::Ice::Long us;

    bool operator==(const CASTTime&) const;
    bool operator<(const CASTTime&) const;
    bool operator!=(const CASTTime& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const CASTTime& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const CASTTime& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const CASTTime& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

enum TaskOutcome
{
    ProcessingIncomplete,
    ProcessingComplete,
    ProcessingCompleteSuccess,
    ProcessingCompleteFailure
};

void __write(::IceInternal::BasicStream*, TaskOutcome);
void __read(::IceInternal::BasicStream*, TaskOutcome&);

enum TaskManagementDecision
{
    TaskAdopted,
    TaskRejected,
    TaskWaiting
};

void __write(::IceInternal::BasicStream*, TaskManagementDecision);
void __read(::IceInternal::BasicStream*, TaskManagementDecision&);

namespace testing
{

const ::Ice::Long CASTTESTPASS = ICE_INT64(29);

const ::Ice::Long CASTTESTFAIL = ICE_INT64(30);

}

}

class CASTException : public ::Ice::UserException
{
public:

    CASTException() {}
    explicit CASTException(const ::std::string&);
    virtual ~CASTException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string message;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

static CASTException __CASTException_init;

class ComponentCreationException : public ::cast::CASTException
{
public:

    ComponentCreationException() {}
    explicit ComponentCreationException(const ::std::string&);
    virtual ~ComponentCreationException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class SubarchitectureComponentException : public ::cast::CASTException
{
public:

    SubarchitectureComponentException() {}
    explicit SubarchitectureComponentException(const ::std::string&);
    virtual ~SubarchitectureComponentException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class UnknownSubarchitectureException : public ::cast::SubarchitectureComponentException
{
public:

    UnknownSubarchitectureException() {}
    UnknownSubarchitectureException(const ::std::string&, const ::std::string&);
    virtual ~UnknownSubarchitectureException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string subarchitecture;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class WMException : public ::cast::SubarchitectureComponentException
{
public:

    WMException() {}
    WMException(const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&);
    virtual ~WMException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::cast::cdl::WorkingMemoryAddress wma;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class DoesNotExistOnWMException : public ::cast::WMException
{
public:

    DoesNotExistOnWMException() {}
    DoesNotExistOnWMException(const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&);
    virtual ~DoesNotExistOnWMException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class AlreadyExistsOnWMException : public ::cast::WMException
{
public:

    AlreadyExistsOnWMException() {}
    AlreadyExistsOnWMException(const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&);
    virtual ~AlreadyExistsOnWMException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class ConsistencyException : public ::cast::WMException
{
public:

    ConsistencyException() {}
    ConsistencyException(const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&);
    virtual ~ConsistencyException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class PermissionException : public ::cast::WMException
{
public:

    PermissionException() {}
    PermissionException(const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&);
    virtual ~PermissionException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

namespace IceProxy
{

namespace cast
{

namespace cdl
{

class TestStructString : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<TestStructString> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructString> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructString*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TestStructString*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class TestStructInt : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestStructInt> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestStructInt*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TestStructInt*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class WorkingMemoryEntry : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryEntry> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryEntry*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<WorkingMemoryEntry*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

namespace testing
{

class CASTTestStruct : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTTestStruct> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTTestStruct*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CASTTestStruct*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class TestDummyStruct : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestDummyStruct> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestDummyStruct*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TestDummyStruct*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace interfaces
{

class TimeServer : virtual public ::IceProxy::Ice::Object
{
public:

    ::cast::cdl::CASTTime getCASTTime()
    {
        return getCASTTime(0);
    }
    ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context& __ctx)
    {
        return getCASTTime(&__ctx);
    }
    
private:

    ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*);
    
public:

    ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double todsecs)
    {
        return fromTimeOfDayDouble(todsecs, 0);
    }
    ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double todsecs, const ::Ice::Context& __ctx)
    {
        return fromTimeOfDayDouble(todsecs, &__ctx);
    }
    
private:

    ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double, const ::Ice::Context*);
    
public:

    ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long secs, ::Ice::Long usecs)
    {
        return fromTimeOfDay(secs, usecs, 0);
    }
    ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long secs, ::Ice::Long usecs, const ::Ice::Context& __ctx)
    {
        return fromTimeOfDay(secs, usecs, &__ctx);
    }
    
private:

    ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long, ::Ice::Long, const ::Ice::Context*);
    
public:

    void reset()
    {
        reset(0);
    }
    void reset(const ::Ice::Context& __ctx)
    {
        reset(&__ctx);
    }
    
private:

    void reset(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<TimeServer> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeServer> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeServer*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TimeServer*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class CASTComponent : virtual public ::IceProxy::Ice::Object
{
public:

    void beat()
    {
        beat(0);
    }
    void beat(const ::Ice::Context& __ctx)
    {
        beat(&__ctx);
    }
    
private:

    void beat(const ::Ice::Context*);
    
public:

    void setID(const ::std::string& id)
    {
        setID(id, 0);
    }
    void setID(const ::std::string& id, const ::Ice::Context& __ctx)
    {
        setID(id, &__ctx);
    }
    
private:

    void setID(const ::std::string&, const ::Ice::Context*);
    
public:

    ::std::string getID()
    {
        return getID(0);
    }
    ::std::string getID(const ::Ice::Context& __ctx)
    {
        return getID(&__ctx);
    }
    
private:

    ::std::string getID(const ::Ice::Context*);
    
public:

    void configure(const ::cast::cdl::StringMap& config)
    {
        configure(config, 0);
    }
    void configure(const ::cast::cdl::StringMap& config, const ::Ice::Context& __ctx)
    {
        configure(config, &__ctx);
    }
    
private:

    void configure(const ::cast::cdl::StringMap&, const ::Ice::Context*);
    
public:

    void start()
    {
        start(0);
    }
    void start(const ::Ice::Context& __ctx)
    {
        start(&__ctx);
    }
    
private:

    void start(const ::Ice::Context*);
    
public:

    void run()
    {
        run(0);
    }
    void run(const ::Ice::Context& __ctx)
    {
        run(&__ctx);
    }
    
private:

    void run(const ::Ice::Context*);
    
public:

    void stop()
    {
        stop(0);
    }
    void stop(const ::Ice::Context& __ctx)
    {
        stop(&__ctx);
    }
    
private:

    void stop(const ::Ice::Context*);
    
public:

    void setComponentManager(const ::cast::interfaces::ComponentManagerPrx& man)
    {
        setComponentManager(man, 0);
    }
    void setComponentManager(const ::cast::interfaces::ComponentManagerPrx& man, const ::Ice::Context& __ctx)
    {
        setComponentManager(man, &__ctx);
    }
    
private:

    void setComponentManager(const ::cast::interfaces::ComponentManagerPrx&, const ::Ice::Context*);
    
public:

    void setTimeServer(const ::cast::interfaces::TimeServerPrx& ts)
    {
        setTimeServer(ts, 0);
    }
    void setTimeServer(const ::cast::interfaces::TimeServerPrx& ts, const ::Ice::Context& __ctx)
    {
        setTimeServer(ts, &__ctx);
    }
    
private:

    void setTimeServer(const ::cast::interfaces::TimeServerPrx&, const ::Ice::Context*);
    
public:

    void destroy()
    {
        destroy(0);
    }
    void destroy(const ::Ice::Context& __ctx)
    {
        destroy(&__ctx);
    }
    
private:

    void destroy(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CASTComponent> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CASTComponent*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CASTComponent*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class WorkingMemoryAttachedComponent : virtual public ::IceProxy::cast::interfaces::CASTComponent
{
public:

    void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm)
    {
        setWorkingMemory(wm, 0);
    }
    void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::Ice::Context& __ctx)
    {
        setWorkingMemory(wm, &__ctx);
    }
    
private:

    void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryAttachedComponent> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryAttachedComponent*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<WorkingMemoryAttachedComponent*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class WorkingMemoryReaderComponent : virtual public ::IceProxy::cast::interfaces::WorkingMemoryAttachedComponent
{
public:

    void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc)
    {
        receiveChangeEvent(wmc, 0);
    }
    void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context& __ctx)
    {
        receiveChangeEvent(wmc, &__ctx);
    }
    
private:

    void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemoryReaderComponent> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemoryReaderComponent*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<WorkingMemoryReaderComponent*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ManagedComponent : virtual public ::IceProxy::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    void setTaskManager(const ::cast::interfaces::TaskManagerPrx& tm)
    {
        setTaskManager(tm, 0);
    }
    void setTaskManager(const ::cast::interfaces::TaskManagerPrx& tm, const ::Ice::Context& __ctx)
    {
        setTaskManager(tm, &__ctx);
    }
    
private:

    void setTaskManager(const ::cast::interfaces::TaskManagerPrx&, const ::Ice::Context*);
    
public:

    void taskDecision(const ::std::string& id, ::cast::cdl::TaskManagementDecision decision)
    {
        taskDecision(id, decision, 0);
    }
    void taskDecision(const ::std::string& id, ::cast::cdl::TaskManagementDecision decision, const ::Ice::Context& __ctx)
    {
        taskDecision(id, decision, &__ctx);
    }
    
private:

    void taskDecision(const ::std::string&, ::cast::cdl::TaskManagementDecision, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ManagedComponent> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ManagedComponent*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ManagedComponent*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class UnmanagedComponent : virtual public ::IceProxy::cast::interfaces::WorkingMemoryAttachedComponent
{
public:
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<UnmanagedComponent> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<UnmanagedComponent*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<UnmanagedComponent*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class TaskManager : virtual public ::IceProxy::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    void proposeTask(const ::std::string& component, const ::std::string& taskID, const ::std::string& taskName)
    {
        proposeTask(component, taskID, taskName, 0);
    }
    void proposeTask(const ::std::string& component, const ::std::string& taskID, const ::std::string& taskName, const ::Ice::Context& __ctx)
    {
        proposeTask(component, taskID, taskName, &__ctx);
    }
    
private:

    void proposeTask(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    void retractTask(const ::std::string& component, const ::std::string& taskID)
    {
        retractTask(component, taskID, 0);
    }
    void retractTask(const ::std::string& component, const ::std::string& taskID, const ::Ice::Context& __ctx)
    {
        retractTask(component, taskID, &__ctx);
    }
    
private:

    void retractTask(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    void taskComplete(const ::std::string& component, const ::std::string& taskID, ::cast::cdl::TaskOutcome outcome)
    {
        taskComplete(component, taskID, outcome, 0);
    }
    void taskComplete(const ::std::string& component, const ::std::string& taskID, ::cast::cdl::TaskOutcome outcome, const ::Ice::Context& __ctx)
    {
        taskComplete(component, taskID, outcome, &__ctx);
    }
    
private:

    void taskComplete(const ::std::string&, const ::std::string&, ::cast::cdl::TaskOutcome, const ::Ice::Context*);
    
public:

    void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx& comp)
    {
        addManagedComponent(comp, 0);
    }
    void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx& comp, const ::Ice::Context& __ctx)
    {
        addManagedComponent(comp, &__ctx);
    }
    
private:

    void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<TaskManager> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TaskManager> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TaskManager*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TaskManager*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class WorkingMemory : virtual public ::IceProxy::cast::interfaces::CASTComponent
{
public:

    bool exists(const ::std::string& id, const ::std::string& subarch)
    {
        return exists(id, subarch, 0);
    }
    bool exists(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context& __ctx)
    {
        return exists(id, subarch, &__ctx);
    }
    
private:

    bool exists(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::Ice::Int getVersionNumber(const ::std::string& id, const ::std::string& subarch)
    {
        return getVersionNumber(id, subarch, 0);
    }
    ::Ice::Int getVersionNumber(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context& __ctx)
    {
        return getVersionNumber(id, subarch, &__ctx);
    }
    
private:

    ::Ice::Int getVersionNumber(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string& id, const ::std::string& subarch)
    {
        return getPermissions(id, subarch, 0);
    }
    ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string& id, const ::std::string& subarch, const ::Ice::Context& __ctx)
    {
        return getPermissions(id, subarch, &__ctx);
    }
    
private:

    ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    void lockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions)
    {
        lockEntry(id, subarch, component, permissions, 0);
    }
    void lockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context& __ctx)
    {
        lockEntry(id, subarch, component, permissions, &__ctx);
    }
    
private:

    void lockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*);
    
public:

    bool tryLockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions)
    {
        return tryLockEntry(id, subarch, component, permissions, 0);
    }
    bool tryLockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, ::cast::cdl::WorkingMemoryPermissions permissions, const ::Ice::Context& __ctx)
    {
        return tryLockEntry(id, subarch, component, permissions, &__ctx);
    }
    
private:

    bool tryLockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*);
    
public:

    void unlockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component)
    {
        unlockEntry(id, subarch, component, 0);
    }
    void unlockEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context& __ctx)
    {
        unlockEntry(id, subarch, component, &__ctx);
    }
    
private:

    void unlockEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::std::string& subarch)
    {
        setWorkingMemory(wm, subarch, 0);
    }
    void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx& wm, const ::std::string& subarch, const ::Ice::Context& __ctx)
    {
        setWorkingMemory(wm, subarch, &__ctx);
    }
    
private:

    void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::std::string&, const ::Ice::Context*);
    
public:

    void addToWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry)
    {
        addToWorkingMemory(id, subarch, type, component, entry, 0);
    }
    void addToWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context& __ctx)
    {
        addToWorkingMemory(id, subarch, type, component, entry, &__ctx);
    }
    
private:

    void addToWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*);
    
public:

    void overwriteWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry)
    {
        overwriteWorkingMemory(id, subarch, type, component, entry, 0);
    }
    void overwriteWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& type, const ::std::string& component, const ::Ice::ObjectPtr& entry, const ::Ice::Context& __ctx)
    {
        overwriteWorkingMemory(id, subarch, type, component, entry, &__ctx);
    }
    
private:

    void overwriteWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*);
    
public:

    void deleteFromWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& component)
    {
        deleteFromWorkingMemory(id, subarch, component, 0);
    }
    void deleteFromWorkingMemory(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context& __ctx)
    {
        deleteFromWorkingMemory(id, subarch, component, &__ctx);
    }
    
private:

    void deleteFromWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component)
    {
        return getWorkingMemoryEntry(id, subarch, component, 0);
    }
    ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string& id, const ::std::string& subarch, const ::std::string& component, const ::Ice::Context& __ctx)
    {
        return getWorkingMemoryEntry(id, subarch, component, &__ctx);
    }
    
private:

    ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    void getWorkingMemoryEntries(const ::std::string& type, const ::std::string& subarch, ::Ice::Int count, const ::std::string& component, ::cast::cdl::WorkingMemoryEntrySeq& entries)
    {
        getWorkingMemoryEntries(type, subarch, count, component, entries, 0);
    }
    void getWorkingMemoryEntries(const ::std::string& type, const ::std::string& subarch, ::Ice::Int count, const ::std::string& component, ::cast::cdl::WorkingMemoryEntrySeq& entries, const ::Ice::Context& __ctx)
    {
        getWorkingMemoryEntries(type, subarch, count, component, entries, &__ctx);
    }
    
private:

    void getWorkingMemoryEntries(const ::std::string&, const ::std::string&, ::Ice::Int, const ::std::string&, ::cast::cdl::WorkingMemoryEntrySeq&, const ::Ice::Context*);
    
public:

    void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter)
    {
        registerComponentFilter(filter, 0);
    }
    void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context& __ctx)
    {
        registerComponentFilter(filter, &__ctx);
    }
    
private:

    void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);
    
public:

    void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter)
    {
        removeComponentFilter(filter, 0);
    }
    void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context& __ctx)
    {
        removeComponentFilter(filter, &__ctx);
    }
    
private:

    void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);
    
public:

    void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::std::string& subarch)
    {
        registerWorkingMemoryFilter(filter, subarch, 0);
    }
    void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::std::string& subarch, const ::Ice::Context& __ctx)
    {
        registerWorkingMemoryFilter(filter, subarch, &__ctx);
    }
    
private:

    void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::std::string&, const ::Ice::Context*);
    
public:

    void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter)
    {
        removeWorkingMemoryFilter(filter, 0);
    }
    void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter& filter, const ::Ice::Context& __ctx)
    {
        removeWorkingMemoryFilter(filter, &__ctx);
    }
    
private:

    void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);
    
public:

    void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx& reader)
    {
        addReader(reader, 0);
    }
    void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx& reader, const ::Ice::Context& __ctx)
    {
        addReader(reader, &__ctx);
    }
    
private:

    void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx&, const ::Ice::Context*);
    
public:

    void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc)
    {
        receiveChangeEvent(wmc, 0);
    }
    void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange& wmc, const ::Ice::Context& __ctx)
    {
        receiveChangeEvent(wmc, &__ctx);
    }
    
private:

    void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WorkingMemory> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WorkingMemory*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<WorkingMemory*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ComponentManager : virtual public ::IceProxy::Ice::Object
{
public:

    ::cast::cdl::CASTTime getCASTTime()
    {
        return getCASTTime(0);
    }
    ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context& __ctx)
    {
        return getCASTTime(&__ctx);
    }
    
private:

    ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentManager> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentManager*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ComponentManager*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ComponentFactory : virtual public ::IceProxy::Ice::Object
{
public:

    ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string& id, const ::std::string& type)
    {
        return newComponent(id, type, 0);
    }
    ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context& __ctx)
    {
        return newComponent(id, type, &__ctx);
    }
    
private:

    ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string& id, const ::std::string& type)
    {
        return newManagedComponent(id, type, 0);
    }
    ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context& __ctx)
    {
        return newManagedComponent(id, type, &__ctx);
    }
    
private:

    ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string& id, const ::std::string& type)
    {
        return newUnmanagedComponent(id, type, 0);
    }
    ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Context& __ctx)
    {
        return newUnmanagedComponent(id, type, &__ctx);
    }
    
private:

    ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string& id, const ::std::string& type)
    {
        return newWorkingMemory(id, type, 0);
    }
    ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string& id, const ::std::string& type, const ::Ice::Context& __ctx)
    {
        return newWorkingMemory(id, type, &__ctx);
    }
    
private:

    ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:

    ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string& id, const ::std::string& type)
    {
        return newTaskManager(id, type, 0);
    }
    ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string& id, const ::std::string& type, const ::Ice::Context& __ctx)
    {
        return newTaskManager(id, type, &__ctx);
    }
    
private:

    ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string&, const ::std::string&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ComponentFactory> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ComponentFactory*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ComponentFactory*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

}

namespace IceDelegate
{

namespace cast
{

namespace cdl
{

class TestStructString : virtual public ::IceDelegate::Ice::Object
{
public:
};

class TestStructInt : virtual public ::IceDelegate::Ice::Object
{
public:
};

class WorkingMemoryEntry : virtual public ::IceDelegate::Ice::Object
{
public:
};

namespace testing
{

class CASTTestStruct : virtual public ::IceDelegate::Ice::Object
{
public:
};

class TestDummyStruct : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

}

namespace interfaces
{

class TimeServer : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*) = 0;

    virtual ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double, const ::Ice::Context*) = 0;

    virtual ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long, ::Ice::Long, const ::Ice::Context*) = 0;

    virtual void reset(const ::Ice::Context*) = 0;
};

class CASTComponent : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void beat(const ::Ice::Context*) = 0;

    virtual void setID(const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::std::string getID(const ::Ice::Context*) = 0;

    virtual void configure(const ::cast::cdl::StringMap&, const ::Ice::Context*) = 0;

    virtual void start(const ::Ice::Context*) = 0;

    virtual void run(const ::Ice::Context*) = 0;

    virtual void stop(const ::Ice::Context*) = 0;

    virtual void setComponentManager(const ::cast::interfaces::ComponentManagerPrx&, const ::Ice::Context*) = 0;

    virtual void setTimeServer(const ::cast::interfaces::TimeServerPrx&, const ::Ice::Context*) = 0;

    virtual void destroy(const ::Ice::Context*) = 0;
};

class WorkingMemoryAttachedComponent : virtual public ::IceDelegate::cast::interfaces::CASTComponent
{
public:

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::Ice::Context*) = 0;
};

class WorkingMemoryReaderComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryAttachedComponent
{
public:

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*) = 0;
};

class ManagedComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    virtual void setTaskManager(const ::cast::interfaces::TaskManagerPrx&, const ::Ice::Context*) = 0;

    virtual void taskDecision(const ::std::string&, ::cast::cdl::TaskManagementDecision, const ::Ice::Context*) = 0;
};

class UnmanagedComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryAttachedComponent
{
public:
};

class TaskManager : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    virtual void proposeTask(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void retractTask(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void taskComplete(const ::std::string&, const ::std::string&, ::cast::cdl::TaskOutcome, const ::Ice::Context*) = 0;

    virtual void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx&, const ::Ice::Context*) = 0;
};

class WorkingMemory : virtual public ::IceDelegate::cast::interfaces::CASTComponent
{
public:

    virtual bool exists(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::Ice::Int getVersionNumber(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void lockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*) = 0;

    virtual bool tryLockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*) = 0;

    virtual void unlockEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void addToWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*) = 0;

    virtual void overwriteWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*) = 0;

    virtual void deleteFromWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void getWorkingMemoryEntries(const ::std::string&, const ::std::string&, ::Ice::Int, const ::std::string&, ::cast::cdl::WorkingMemoryEntrySeq&, const ::Ice::Context*) = 0;

    virtual void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*) = 0;

    virtual void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*) = 0;

    virtual void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*) = 0;

    virtual void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx&, const ::Ice::Context*) = 0;

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*) = 0;
};

class ComponentManager : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*) = 0;
};

class ComponentFactory : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;

    virtual ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string&, const ::std::string&, const ::Ice::Context*) = 0;
};

}

}

}

namespace IceDelegateM
{

namespace cast
{

namespace cdl
{

class TestStructString : virtual public ::IceDelegate::cast::cdl::TestStructString,
                         virtual public ::IceDelegateM::Ice::Object
{
public:
};

class TestStructInt : virtual public ::IceDelegate::cast::cdl::TestStructInt,
                      virtual public ::IceDelegateM::Ice::Object
{
public:
};

class WorkingMemoryEntry : virtual public ::IceDelegate::cast::cdl::WorkingMemoryEntry,
                           virtual public ::IceDelegateM::Ice::Object
{
public:
};

namespace testing
{

class CASTTestStruct : virtual public ::IceDelegate::cast::cdl::testing::CASTTestStruct,
                       virtual public ::IceDelegateM::Ice::Object
{
public:
};

class TestDummyStruct : virtual public ::IceDelegate::cast::cdl::testing::TestDummyStruct,
                        virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

}

namespace interfaces
{

class TimeServer : virtual public ::IceDelegate::cast::interfaces::TimeServer,
                   virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*);

    virtual ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double, const ::Ice::Context*);

    virtual ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long, ::Ice::Long, const ::Ice::Context*);

    virtual void reset(const ::Ice::Context*);
};

class CASTComponent : virtual public ::IceDelegate::cast::interfaces::CASTComponent,
                      virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void beat(const ::Ice::Context*);

    virtual void setID(const ::std::string&, const ::Ice::Context*);

    virtual ::std::string getID(const ::Ice::Context*);

    virtual void configure(const ::cast::cdl::StringMap&, const ::Ice::Context*);

    virtual void start(const ::Ice::Context*);

    virtual void run(const ::Ice::Context*);

    virtual void stop(const ::Ice::Context*);

    virtual void setComponentManager(const ::cast::interfaces::ComponentManagerPrx&, const ::Ice::Context*);

    virtual void setTimeServer(const ::cast::interfaces::TimeServerPrx&, const ::Ice::Context*);

    virtual void destroy(const ::Ice::Context*);
};

class WorkingMemoryAttachedComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryAttachedComponent,
                                       virtual public ::IceDelegateM::cast::interfaces::CASTComponent
{
public:

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::Ice::Context*);
};

class WorkingMemoryReaderComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryReaderComponent,
                                     virtual public ::IceDelegateM::cast::interfaces::WorkingMemoryAttachedComponent
{
public:

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*);
};

class ManagedComponent : virtual public ::IceDelegate::cast::interfaces::ManagedComponent,
                         virtual public ::IceDelegateM::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    virtual void setTaskManager(const ::cast::interfaces::TaskManagerPrx&, const ::Ice::Context*);

    virtual void taskDecision(const ::std::string&, ::cast::cdl::TaskManagementDecision, const ::Ice::Context*);
};

class UnmanagedComponent : virtual public ::IceDelegate::cast::interfaces::UnmanagedComponent,
                           virtual public ::IceDelegateM::cast::interfaces::WorkingMemoryAttachedComponent
{
public:
};

class TaskManager : virtual public ::IceDelegate::cast::interfaces::TaskManager,
                    virtual public ::IceDelegateM::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    virtual void proposeTask(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void retractTask(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void taskComplete(const ::std::string&, const ::std::string&, ::cast::cdl::TaskOutcome, const ::Ice::Context*);

    virtual void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx&, const ::Ice::Context*);
};

class WorkingMemory : virtual public ::IceDelegate::cast::interfaces::WorkingMemory,
                      virtual public ::IceDelegateM::cast::interfaces::CASTComponent
{
public:

    virtual bool exists(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::Ice::Int getVersionNumber(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void lockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*);

    virtual bool tryLockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*);

    virtual void unlockEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::std::string&, const ::Ice::Context*);

    virtual void addToWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*);

    virtual void overwriteWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*);

    virtual void deleteFromWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void getWorkingMemoryEntries(const ::std::string&, const ::std::string&, ::Ice::Int, const ::std::string&, ::cast::cdl::WorkingMemoryEntrySeq&, const ::Ice::Context*);

    virtual void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);

    virtual void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);

    virtual void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::std::string&, const ::Ice::Context*);

    virtual void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);

    virtual void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx&, const ::Ice::Context*);

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*);
};

class ComponentManager : virtual public ::IceDelegate::cast::interfaces::ComponentManager,
                         virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*);
};

class ComponentFactory : virtual public ::IceDelegate::cast::interfaces::ComponentFactory,
                         virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string&, const ::std::string&, const ::Ice::Context*);
};

}

}

}

namespace IceDelegateD
{

namespace cast
{

namespace cdl
{

class TestStructString : virtual public ::IceDelegate::cast::cdl::TestStructString,
                         virtual public ::IceDelegateD::Ice::Object
{
public:
};

class TestStructInt : virtual public ::IceDelegate::cast::cdl::TestStructInt,
                      virtual public ::IceDelegateD::Ice::Object
{
public:
};

class WorkingMemoryEntry : virtual public ::IceDelegate::cast::cdl::WorkingMemoryEntry,
                           virtual public ::IceDelegateD::Ice::Object
{
public:
};

namespace testing
{

class CASTTestStruct : virtual public ::IceDelegate::cast::cdl::testing::CASTTestStruct,
                       virtual public ::IceDelegateD::Ice::Object
{
public:
};

class TestDummyStruct : virtual public ::IceDelegate::cast::cdl::testing::TestDummyStruct,
                        virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

}

namespace interfaces
{

class TimeServer : virtual public ::IceDelegate::cast::interfaces::TimeServer,
                   virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*);

    virtual ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double, const ::Ice::Context*);

    virtual ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long, ::Ice::Long, const ::Ice::Context*);

    virtual void reset(const ::Ice::Context*);
};

class CASTComponent : virtual public ::IceDelegate::cast::interfaces::CASTComponent,
                      virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void beat(const ::Ice::Context*);

    virtual void setID(const ::std::string&, const ::Ice::Context*);

    virtual ::std::string getID(const ::Ice::Context*);

    virtual void configure(const ::cast::cdl::StringMap&, const ::Ice::Context*);

    virtual void start(const ::Ice::Context*);

    virtual void run(const ::Ice::Context*);

    virtual void stop(const ::Ice::Context*);

    virtual void setComponentManager(const ::cast::interfaces::ComponentManagerPrx&, const ::Ice::Context*);

    virtual void setTimeServer(const ::cast::interfaces::TimeServerPrx&, const ::Ice::Context*);

    virtual void destroy(const ::Ice::Context*);
};

class WorkingMemoryAttachedComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryAttachedComponent,
                                       virtual public ::IceDelegateD::cast::interfaces::CASTComponent
{
public:

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::Ice::Context*);
};

class WorkingMemoryReaderComponent : virtual public ::IceDelegate::cast::interfaces::WorkingMemoryReaderComponent,
                                     virtual public ::IceDelegateD::cast::interfaces::WorkingMemoryAttachedComponent
{
public:

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*);
};

class ManagedComponent : virtual public ::IceDelegate::cast::interfaces::ManagedComponent,
                         virtual public ::IceDelegateD::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    virtual void setTaskManager(const ::cast::interfaces::TaskManagerPrx&, const ::Ice::Context*);

    virtual void taskDecision(const ::std::string&, ::cast::cdl::TaskManagementDecision, const ::Ice::Context*);
};

class UnmanagedComponent : virtual public ::IceDelegate::cast::interfaces::UnmanagedComponent,
                           virtual public ::IceDelegateD::cast::interfaces::WorkingMemoryAttachedComponent
{
public:
};

class TaskManager : virtual public ::IceDelegate::cast::interfaces::TaskManager,
                    virtual public ::IceDelegateD::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    virtual void proposeTask(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void retractTask(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void taskComplete(const ::std::string&, const ::std::string&, ::cast::cdl::TaskOutcome, const ::Ice::Context*);

    virtual void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx&, const ::Ice::Context*);
};

class WorkingMemory : virtual public ::IceDelegate::cast::interfaces::WorkingMemory,
                      virtual public ::IceDelegateD::cast::interfaces::CASTComponent
{
public:

    virtual bool exists(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::Ice::Int getVersionNumber(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void lockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*);

    virtual bool tryLockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Context*);

    virtual void unlockEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::std::string&, const ::Ice::Context*);

    virtual void addToWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*);

    virtual void overwriteWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Context*);

    virtual void deleteFromWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual void getWorkingMemoryEntries(const ::std::string&, const ::std::string&, ::Ice::Int, const ::std::string&, ::cast::cdl::WorkingMemoryEntrySeq&, const ::Ice::Context*);

    virtual void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);

    virtual void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);

    virtual void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::std::string&, const ::Ice::Context*);

    virtual void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Context*);

    virtual void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx&, const ::Ice::Context*);

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Context*);
};

class ComponentManager : virtual public ::IceDelegate::cast::interfaces::ComponentManager,
                         virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Context*);
};

class ComponentFactory : virtual public ::IceDelegate::cast::interfaces::ComponentFactory,
                         virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string&, const ::std::string&, const ::Ice::Context*);

    virtual ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string&, const ::std::string&, const ::Ice::Context*);
};

}

}

}

namespace cast
{

namespace cdl
{

class TestStructString : virtual public ::Ice::Object
{
public:

    typedef TestStructStringPrx ProxyType;
    typedef TestStructStringPtr PointerType;
    
    TestStructString() {}
    explicit TestStructString(const ::std::string&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~TestStructString() {}

    friend class TestStructString__staticInit;

public:

    ::std::string dummy;
};

class TestStructString__staticInit
{
public:

    ::cast::cdl::TestStructString _init;
};

static ::cast::cdl::TestStructString__staticInit _TestStructString_init;

class TestStructInt : virtual public ::Ice::Object
{
public:

    typedef TestStructIntPrx ProxyType;
    typedef TestStructIntPtr PointerType;
    
    TestStructInt() {}
    explicit TestStructInt(::Ice::Int);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~TestStructInt() {}

public:

    ::Ice::Int dummy;
};

class WorkingMemoryEntry : virtual public ::Ice::Object
{
public:

    typedef WorkingMemoryEntryPrx ProxyType;
    typedef WorkingMemoryEntryPtr PointerType;
    
    WorkingMemoryEntry() {}
    WorkingMemoryEntry(const ::std::string&, const ::std::string&, ::Ice::Int, const ::Ice::ObjectPtr&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void __incRef();
    virtual void __decRef();
    virtual void __addObject(::IceInternal::GCCountMap&);
    virtual bool __usesClasses();
    virtual void __gcReachable(::IceInternal::GCCountMap&) const;
    virtual void __gcClear();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~WorkingMemoryEntry() {}

public:

    ::std::string id;

    ::std::string type;

    ::Ice::Int version;

    ::Ice::ObjectPtr entry;
};

namespace testing
{

class CASTTestStruct : virtual public ::Ice::Object
{
public:

    typedef CASTTestStructPrx ProxyType;
    typedef CASTTestStructPtr PointerType;
    
    CASTTestStruct() {}
    CASTTestStruct(::Ice::Long, const ::cast::cdl::WorkingMemoryChange&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~CASTTestStruct() {}

public:

    ::Ice::Long count;

    ::cast::cdl::WorkingMemoryChange change;
};

class TestDummyStruct : virtual public ::Ice::Object
{
public:

    typedef TestDummyStructPrx ProxyType;
    typedef TestDummyStructPtr PointerType;
    
    TestDummyStruct() {}
    explicit TestDummyStruct(const ::std::string&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~TestDummyStruct() {}

public:

    ::std::string dummy;
};

}

}

namespace interfaces
{

class TimeServer : virtual public ::Ice::Object
{
public:

    typedef TimeServerPrx ProxyType;
    typedef TimeServerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Current& = ::Ice::Current()) const = 0;
    ::Ice::DispatchStatus ___getCASTTime(::IceInternal::Incoming&, const ::Ice::Current&) const;

    virtual ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double, const ::Ice::Current& = ::Ice::Current()) const = 0;
    ::Ice::DispatchStatus ___fromTimeOfDayDouble(::IceInternal::Incoming&, const ::Ice::Current&) const;

    virtual ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long, ::Ice::Long, const ::Ice::Current& = ::Ice::Current()) const = 0;
    ::Ice::DispatchStatus ___fromTimeOfDay(::IceInternal::Incoming&, const ::Ice::Current&) const;

    virtual void reset(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___reset(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class CASTComponent : virtual public ::Ice::Object
{
public:

    typedef CASTComponentPrx ProxyType;
    typedef CASTComponentPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void beat(const ::Ice::Current& = ::Ice::Current()) const = 0;
    ::Ice::DispatchStatus ___beat(::IceInternal::Incoming&, const ::Ice::Current&) const;

    virtual void setID(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setID(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::std::string getID(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getID(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void configure(const ::cast::cdl::StringMap&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___configure(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void start(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___start(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void run(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___run(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void stop(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___stop(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setComponentManager(const ::cast::interfaces::ComponentManagerPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setComponentManager(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setTimeServer(const ::cast::interfaces::TimeServerPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setTimeServer(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void destroy(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___destroy(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class WorkingMemoryAttachedComponent : virtual public ::cast::interfaces::CASTComponent
{
public:

    typedef WorkingMemoryAttachedComponentPrx ProxyType;
    typedef WorkingMemoryAttachedComponentPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setWorkingMemory(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class WorkingMemoryReaderComponent : virtual public ::cast::interfaces::WorkingMemoryAttachedComponent
{
public:

    typedef WorkingMemoryReaderComponentPrx ProxyType;
    typedef WorkingMemoryReaderComponentPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___receiveChangeEvent(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class ManagedComponent : virtual public ::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    typedef ManagedComponentPrx ProxyType;
    typedef ManagedComponentPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void setTaskManager(const ::cast::interfaces::TaskManagerPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setTaskManager(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void taskDecision(const ::std::string&, ::cast::cdl::TaskManagementDecision, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___taskDecision(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class UnmanagedComponent : virtual public ::cast::interfaces::WorkingMemoryAttachedComponent
{
public:

    typedef UnmanagedComponentPrx ProxyType;
    typedef UnmanagedComponentPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class TaskManager : virtual public ::cast::interfaces::WorkingMemoryReaderComponent
{
public:

    typedef TaskManagerPrx ProxyType;
    typedef TaskManagerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void proposeTask(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___proposeTask(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void retractTask(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___retractTask(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void taskComplete(const ::std::string&, const ::std::string&, ::cast::cdl::TaskOutcome, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___taskComplete(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___addManagedComponent(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class WorkingMemory : virtual public ::cast::interfaces::CASTComponent
{
public:

    typedef WorkingMemoryPrx ProxyType;
    typedef WorkingMemoryPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual bool exists(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___exists(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::Int getVersionNumber(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getVersionNumber(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::cast::cdl::WorkingMemoryPermissions getPermissions(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPermissions(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void lockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___lockEntry(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual bool tryLockEntry(const ::std::string&, const ::std::string&, const ::std::string&, ::cast::cdl::WorkingMemoryPermissions, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___tryLockEntry(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void unlockEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___unlockEntry(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setWorkingMemory(const ::cast::interfaces::WorkingMemoryPrx&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setWorkingMemory(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void addToWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___addToWorkingMemory(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void overwriteWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::ObjectPtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___overwriteWorkingMemory(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void deleteFromWorkingMemory(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___deleteFromWorkingMemory(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::cast::cdl::WorkingMemoryEntryPtr getWorkingMemoryEntry(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getWorkingMemoryEntry(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getWorkingMemoryEntries(const ::std::string&, const ::std::string&, ::Ice::Int, const ::std::string&, ::cast::cdl::WorkingMemoryEntrySeq&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getWorkingMemoryEntries(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void registerComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___registerComponentFilter(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void removeComponentFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___removeComponentFilter(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void registerWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___registerWorkingMemoryFilter(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void removeWorkingMemoryFilter(const ::cast::cdl::WorkingMemoryChangeFilter&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___removeWorkingMemoryFilter(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void addReader(const ::cast::interfaces::WorkingMemoryReaderComponentPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___addReader(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void receiveChangeEvent(const ::cast::cdl::WorkingMemoryChange&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___receiveChangeEvent(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class ComponentManager : virtual public ::Ice::Object
{
public:

    typedef ComponentManagerPrx ProxyType;
    typedef ComponentManagerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Current& = ::Ice::Current()) const = 0;
    ::Ice::DispatchStatus ___getCASTTime(::IceInternal::Incoming&, const ::Ice::Current&) const;

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class ComponentFactory : virtual public ::Ice::Object
{
public:

    typedef ComponentFactoryPrx ProxyType;
    typedef ComponentFactoryPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___newComponent(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___newManagedComponent(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___newUnmanagedComponent(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___newWorkingMemory(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string&, const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___newTaskManager(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

}

#endif
