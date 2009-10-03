// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `Execution.ice'

#ifndef ___home_marc_eclipse_bham_system_subarchitectures_motivation_sa_src_c___autogen_Execution_hpp__
#define ___home_marc_eclipse_bham_system_subarchitectures_motivation_sa_src_c___autogen_Execution_hpp__

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
#include <cast/slice/CDL.hpp>
#include <Planner.hpp>
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

namespace execution
{

namespace slice
{

class Action;

namespace actions
{

class GoToPlace;

class ActiveVisualSearch;

class ExplorePlace;

class PrintMessage;

class LogMessage;

}

}

}

}

namespace execution
{

namespace slice
{

class Action;
bool operator==(const Action&, const Action&);
bool operator<(const Action&, const Action&);

namespace actions
{

class GoToPlace;
bool operator==(const GoToPlace&, const GoToPlace&);
bool operator<(const GoToPlace&, const GoToPlace&);

class ActiveVisualSearch;
bool operator==(const ActiveVisualSearch&, const ActiveVisualSearch&);
bool operator<(const ActiveVisualSearch&, const ActiveVisualSearch&);

class ExplorePlace;
bool operator==(const ExplorePlace&, const ExplorePlace&);
bool operator<(const ExplorePlace&, const ExplorePlace&);

class PrintMessage;
bool operator==(const PrintMessage&, const PrintMessage&);
bool operator<(const PrintMessage&, const PrintMessage&);

class LogMessage;
bool operator==(const LogMessage&, const LogMessage&);
bool operator<(const LogMessage&, const LogMessage&);

}

}

}

namespace IceInternal
{

::Ice::Object* upCast(::execution::slice::Action*);
::IceProxy::Ice::Object* upCast(::IceProxy::execution::slice::Action*);

::Ice::Object* upCast(::execution::slice::actions::GoToPlace*);
::IceProxy::Ice::Object* upCast(::IceProxy::execution::slice::actions::GoToPlace*);

::Ice::Object* upCast(::execution::slice::actions::ActiveVisualSearch*);
::IceProxy::Ice::Object* upCast(::IceProxy::execution::slice::actions::ActiveVisualSearch*);

::Ice::Object* upCast(::execution::slice::actions::ExplorePlace*);
::IceProxy::Ice::Object* upCast(::IceProxy::execution::slice::actions::ExplorePlace*);

::Ice::Object* upCast(::execution::slice::actions::PrintMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::execution::slice::actions::PrintMessage*);

::Ice::Object* upCast(::execution::slice::actions::LogMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::execution::slice::actions::LogMessage*);

}

namespace execution
{

namespace slice
{

typedef ::IceInternal::Handle< ::execution::slice::Action> ActionPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::execution::slice::Action> ActionPrx;

void __read(::IceInternal::BasicStream*, ActionPrx&);
void __patch__ActionPtr(void*, ::Ice::ObjectPtr&);

void ice_writeActionPrx(const ::Ice::OutputStreamPtr&, const ActionPrx&);
void ice_readActionPrx(const ::Ice::InputStreamPtr&, ActionPrx&);
void ice_writeAction(const ::Ice::OutputStreamPtr&, const ActionPtr&);
void ice_readAction(const ::Ice::InputStreamPtr&, ActionPtr&);

namespace actions
{

typedef ::IceInternal::Handle< ::execution::slice::actions::GoToPlace> GoToPlacePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::execution::slice::actions::GoToPlace> GoToPlacePrx;

void __read(::IceInternal::BasicStream*, GoToPlacePrx&);
void __patch__GoToPlacePtr(void*, ::Ice::ObjectPtr&);

void ice_writeGoToPlacePrx(const ::Ice::OutputStreamPtr&, const GoToPlacePrx&);
void ice_readGoToPlacePrx(const ::Ice::InputStreamPtr&, GoToPlacePrx&);
void ice_writeGoToPlace(const ::Ice::OutputStreamPtr&, const GoToPlacePtr&);
void ice_readGoToPlace(const ::Ice::InputStreamPtr&, GoToPlacePtr&);

typedef ::IceInternal::Handle< ::execution::slice::actions::ActiveVisualSearch> ActiveVisualSearchPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::execution::slice::actions::ActiveVisualSearch> ActiveVisualSearchPrx;

void __read(::IceInternal::BasicStream*, ActiveVisualSearchPrx&);
void __patch__ActiveVisualSearchPtr(void*, ::Ice::ObjectPtr&);

void ice_writeActiveVisualSearchPrx(const ::Ice::OutputStreamPtr&, const ActiveVisualSearchPrx&);
void ice_readActiveVisualSearchPrx(const ::Ice::InputStreamPtr&, ActiveVisualSearchPrx&);
void ice_writeActiveVisualSearch(const ::Ice::OutputStreamPtr&, const ActiveVisualSearchPtr&);
void ice_readActiveVisualSearch(const ::Ice::InputStreamPtr&, ActiveVisualSearchPtr&);

typedef ::IceInternal::Handle< ::execution::slice::actions::ExplorePlace> ExplorePlacePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::execution::slice::actions::ExplorePlace> ExplorePlacePrx;

void __read(::IceInternal::BasicStream*, ExplorePlacePrx&);
void __patch__ExplorePlacePtr(void*, ::Ice::ObjectPtr&);

void ice_writeExplorePlacePrx(const ::Ice::OutputStreamPtr&, const ExplorePlacePrx&);
void ice_readExplorePlacePrx(const ::Ice::InputStreamPtr&, ExplorePlacePrx&);
void ice_writeExplorePlace(const ::Ice::OutputStreamPtr&, const ExplorePlacePtr&);
void ice_readExplorePlace(const ::Ice::InputStreamPtr&, ExplorePlacePtr&);

typedef ::IceInternal::Handle< ::execution::slice::actions::PrintMessage> PrintMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::execution::slice::actions::PrintMessage> PrintMessagePrx;

void __read(::IceInternal::BasicStream*, PrintMessagePrx&);
void __patch__PrintMessagePtr(void*, ::Ice::ObjectPtr&);

void ice_writePrintMessagePrx(const ::Ice::OutputStreamPtr&, const PrintMessagePrx&);
void ice_readPrintMessagePrx(const ::Ice::InputStreamPtr&, PrintMessagePrx&);
void ice_writePrintMessage(const ::Ice::OutputStreamPtr&, const PrintMessagePtr&);
void ice_readPrintMessage(const ::Ice::InputStreamPtr&, PrintMessagePtr&);

typedef ::IceInternal::Handle< ::execution::slice::actions::LogMessage> LogMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::execution::slice::actions::LogMessage> LogMessagePrx;

void __read(::IceInternal::BasicStream*, LogMessagePrx&);
void __patch__LogMessagePtr(void*, ::Ice::ObjectPtr&);

void ice_writeLogMessagePrx(const ::Ice::OutputStreamPtr&, const LogMessagePrx&);
void ice_readLogMessagePrx(const ::Ice::InputStreamPtr&, LogMessagePrx&);
void ice_writeLogMessage(const ::Ice::OutputStreamPtr&, const LogMessagePtr&);
void ice_readLogMessage(const ::Ice::InputStreamPtr&, LogMessagePtr&);

}

}

}

namespace execution
{

namespace slice
{

enum TriBool
{
    TRITRUE,
    TRIFALSE,
    TRIINDETERMINATE
};

void __write(::IceInternal::BasicStream*, TriBool);
void __read(::IceInternal::BasicStream*, TriBool&);

void ice_writeTriBool(const ::Ice::OutputStreamPtr&, TriBool);
void ice_readTriBool(const ::Ice::InputStreamPtr&, TriBool&);

enum ActionStatus
{
    PENDING,
    ACCEPTED,
    INPROGRESS,
    COMPLETE
};

void __write(::IceInternal::BasicStream*, ActionStatus);
void __read(::IceInternal::BasicStream*, ActionStatus&);

void ice_writeActionStatus(const ::Ice::OutputStreamPtr&, ActionStatus);
void ice_readActionStatus(const ::Ice::InputStreamPtr&, ActionStatus&);

class ActionExecutionException : public ::cast::CASTException
{
public:

    ActionExecutionException() {}
    explicit ActionExecutionException(const ::std::string&);
    virtual ~ActionExecutionException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

static ActionExecutionException __ActionExecutionException_init;

namespace actions
{

typedef ::std::vector< ::Ice::Long> LongSeq;

}

}

}

namespace IceProxy
{

namespace execution
{

namespace slice
{

class Action : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Action> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Action> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Action*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Action*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

namespace actions
{

class GoToPlace : virtual public ::IceProxy::execution::slice::Action
{
public:
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<GoToPlace> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<GoToPlace*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<GoToPlace*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ActiveVisualSearch : virtual public ::IceProxy::execution::slice::Action
{
public:
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ActiveVisualSearch> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ActiveVisualSearch*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ActiveVisualSearch*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ExplorePlace : virtual public ::IceProxy::execution::slice::Action
{
public:
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExplorePlace> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExplorePlace*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ExplorePlace*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class PrintMessage : virtual public ::IceProxy::execution::slice::Action
{
public:
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PrintMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PrintMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<PrintMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class LogMessage : virtual public ::IceProxy::execution::slice::Action
{
public:
    
    ::IceInternal::ProxyHandle<LogMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LogMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LogMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<LogMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

}

namespace IceDelegate
{

namespace execution
{

namespace slice
{

class Action : virtual public ::IceDelegate::Ice::Object
{
public:
};

namespace actions
{

class GoToPlace : virtual public ::IceDelegate::execution::slice::Action
{
public:
};

class ActiveVisualSearch : virtual public ::IceDelegate::execution::slice::Action
{
public:
};

class ExplorePlace : virtual public ::IceDelegate::execution::slice::Action
{
public:
};

class PrintMessage : virtual public ::IceDelegate::execution::slice::Action
{
public:
};

class LogMessage : virtual public ::IceDelegate::execution::slice::Action
{
public:
};

}

}

}

}

namespace IceDelegateM
{

namespace execution
{

namespace slice
{

class Action : virtual public ::IceDelegate::execution::slice::Action,
               virtual public ::IceDelegateM::Ice::Object
{
public:
};

namespace actions
{

class GoToPlace : virtual public ::IceDelegate::execution::slice::actions::GoToPlace,
                  virtual public ::IceDelegateM::execution::slice::Action
{
public:
};

class ActiveVisualSearch : virtual public ::IceDelegate::execution::slice::actions::ActiveVisualSearch,
                           virtual public ::IceDelegateM::execution::slice::Action
{
public:
};

class ExplorePlace : virtual public ::IceDelegate::execution::slice::actions::ExplorePlace,
                     virtual public ::IceDelegateM::execution::slice::Action
{
public:
};

class PrintMessage : virtual public ::IceDelegate::execution::slice::actions::PrintMessage,
                     virtual public ::IceDelegateM::execution::slice::Action
{
public:
};

class LogMessage : virtual public ::IceDelegate::execution::slice::actions::LogMessage,
                   virtual public ::IceDelegateM::execution::slice::Action
{
public:
};

}

}

}

}

namespace IceDelegateD
{

namespace execution
{

namespace slice
{

class Action : virtual public ::IceDelegate::execution::slice::Action,
               virtual public ::IceDelegateD::Ice::Object
{
public:
};

namespace actions
{

class GoToPlace : virtual public ::IceDelegate::execution::slice::actions::GoToPlace,
                  virtual public ::IceDelegateD::execution::slice::Action
{
public:
};

class ActiveVisualSearch : virtual public ::IceDelegate::execution::slice::actions::ActiveVisualSearch,
                           virtual public ::IceDelegateD::execution::slice::Action
{
public:
};

class ExplorePlace : virtual public ::IceDelegate::execution::slice::actions::ExplorePlace,
                     virtual public ::IceDelegateD::execution::slice::Action
{
public:
};

class PrintMessage : virtual public ::IceDelegate::execution::slice::actions::PrintMessage,
                     virtual public ::IceDelegateD::execution::slice::Action
{
public:
};

class LogMessage : virtual public ::IceDelegate::execution::slice::actions::LogMessage,
                   virtual public ::IceDelegateD::execution::slice::Action
{
public:
};

}

}

}

}

namespace execution
{

namespace slice
{

class Action : virtual public ::Ice::Object
{
public:

    typedef ActionPrx ProxyType;
    typedef ActionPtr PointerType;
    
    Action() {}
    Action(::execution::slice::ActionStatus, ::execution::slice::TriBool);
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

    virtual ~Action() {}

    friend class Action__staticInit;

public:

    ::execution::slice::ActionStatus status;

    ::execution::slice::TriBool success;
};

class Action__staticInit
{
public:

    ::execution::slice::Action _init;
};

static ::execution::slice::Action__staticInit _Action_init;

namespace actions
{

class GoToPlace : virtual public ::execution::slice::Action
{
public:

    typedef GoToPlacePrx ProxyType;
    typedef GoToPlacePtr PointerType;
    
    GoToPlace() {}
    GoToPlace(::execution::slice::ActionStatus, ::execution::slice::TriBool, ::Ice::Long);
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

    virtual ~GoToPlace() {}

public:

    ::Ice::Long placeID;
};

class ActiveVisualSearch : virtual public ::execution::slice::Action
{
public:

    typedef ActiveVisualSearchPrx ProxyType;
    typedef ActiveVisualSearchPtr PointerType;
    
    ActiveVisualSearch() {}
    ActiveVisualSearch(::execution::slice::ActionStatus, ::execution::slice::TriBool, const ::execution::slice::actions::LongSeq&);
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

    virtual ~ActiveVisualSearch() {}

public:

    ::execution::slice::actions::LongSeq placeIDs;
};

class ExplorePlace : virtual public ::execution::slice::Action
{
public:

    typedef ExplorePlacePrx ProxyType;
    typedef ExplorePlacePtr PointerType;
    
    ExplorePlace() {}
    ExplorePlace(::execution::slice::ActionStatus, ::execution::slice::TriBool);
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

    virtual ~ExplorePlace() {}
};

class PrintMessage : virtual public ::execution::slice::Action
{
public:

    typedef PrintMessagePrx ProxyType;
    typedef PrintMessagePtr PointerType;
    
    PrintMessage() {}
    PrintMessage(::execution::slice::ActionStatus, ::execution::slice::TriBool, const ::std::string&);
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

    virtual ~PrintMessage() {}

public:

    ::std::string message;
};

class LogMessage : virtual public ::execution::slice::Action
{
public:

    typedef LogMessagePrx ProxyType;
    typedef LogMessagePtr PointerType;
    
    LogMessage() {}
    LogMessage(::execution::slice::ActionStatus, ::execution::slice::TriBool, const ::std::string&);
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

    virtual ~LogMessage() {}

public:

    ::std::string message;
};

}

}

}

#endif
