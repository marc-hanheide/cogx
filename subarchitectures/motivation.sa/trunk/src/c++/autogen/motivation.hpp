// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `motivation.ice'

#ifndef ___home_marc_eclipse_bham_system_subarchitectures_motivation_sa_src_c___autogen_motivation_hpp__
#define ___home_marc_eclipse_bham_system_subarchitectures_motivation_sa_src_c___autogen_motivation_hpp__

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

namespace motivation
{

namespace slice
{

class TestSource;

class Motive;

class TestMotive;

class HomingMotive;

class ExploreMotive;

class CategorizePlaceMotive;

class CategorizeRoomMotive;

class PlanProxy;

}

}

}

namespace motivation
{

namespace slice
{

class TestSource;
bool operator==(const TestSource&, const TestSource&);
bool operator<(const TestSource&, const TestSource&);

class Motive;
bool operator==(const Motive&, const Motive&);
bool operator<(const Motive&, const Motive&);

class TestMotive;
bool operator==(const TestMotive&, const TestMotive&);
bool operator<(const TestMotive&, const TestMotive&);

class HomingMotive;
bool operator==(const HomingMotive&, const HomingMotive&);
bool operator<(const HomingMotive&, const HomingMotive&);

class ExploreMotive;
bool operator==(const ExploreMotive&, const ExploreMotive&);
bool operator<(const ExploreMotive&, const ExploreMotive&);

class CategorizePlaceMotive;
bool operator==(const CategorizePlaceMotive&, const CategorizePlaceMotive&);
bool operator<(const CategorizePlaceMotive&, const CategorizePlaceMotive&);

class CategorizeRoomMotive;
bool operator==(const CategorizeRoomMotive&, const CategorizeRoomMotive&);
bool operator<(const CategorizeRoomMotive&, const CategorizeRoomMotive&);

class PlanProxy;
bool operator==(const PlanProxy&, const PlanProxy&);
bool operator<(const PlanProxy&, const PlanProxy&);

}

}

namespace IceInternal
{

::Ice::Object* upCast(::motivation::slice::TestSource*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::TestSource*);

::Ice::Object* upCast(::motivation::slice::Motive*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::Motive*);

::Ice::Object* upCast(::motivation::slice::TestMotive*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::TestMotive*);

::Ice::Object* upCast(::motivation::slice::HomingMotive*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::HomingMotive*);

::Ice::Object* upCast(::motivation::slice::ExploreMotive*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::ExploreMotive*);

::Ice::Object* upCast(::motivation::slice::CategorizePlaceMotive*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::CategorizePlaceMotive*);

::Ice::Object* upCast(::motivation::slice::CategorizeRoomMotive*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::CategorizeRoomMotive*);

::Ice::Object* upCast(::motivation::slice::PlanProxy*);
::IceProxy::Ice::Object* upCast(::IceProxy::motivation::slice::PlanProxy*);

}

namespace motivation
{

namespace slice
{

typedef ::IceInternal::Handle< ::motivation::slice::TestSource> TestSourcePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::TestSource> TestSourcePrx;

void __read(::IceInternal::BasicStream*, TestSourcePrx&);
void __patch__TestSourcePtr(void*, ::Ice::ObjectPtr&);

void ice_writeTestSourcePrx(const ::Ice::OutputStreamPtr&, const TestSourcePrx&);
void ice_readTestSourcePrx(const ::Ice::InputStreamPtr&, TestSourcePrx&);
void ice_writeTestSource(const ::Ice::OutputStreamPtr&, const TestSourcePtr&);
void ice_readTestSource(const ::Ice::InputStreamPtr&, TestSourcePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::Motive> MotivePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::Motive> MotivePrx;

void __read(::IceInternal::BasicStream*, MotivePrx&);
void __patch__MotivePtr(void*, ::Ice::ObjectPtr&);

void ice_writeMotivePrx(const ::Ice::OutputStreamPtr&, const MotivePrx&);
void ice_readMotivePrx(const ::Ice::InputStreamPtr&, MotivePrx&);
void ice_writeMotive(const ::Ice::OutputStreamPtr&, const MotivePtr&);
void ice_readMotive(const ::Ice::InputStreamPtr&, MotivePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::TestMotive> TestMotivePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::TestMotive> TestMotivePrx;

void __read(::IceInternal::BasicStream*, TestMotivePrx&);
void __patch__TestMotivePtr(void*, ::Ice::ObjectPtr&);

void ice_writeTestMotivePrx(const ::Ice::OutputStreamPtr&, const TestMotivePrx&);
void ice_readTestMotivePrx(const ::Ice::InputStreamPtr&, TestMotivePrx&);
void ice_writeTestMotive(const ::Ice::OutputStreamPtr&, const TestMotivePtr&);
void ice_readTestMotive(const ::Ice::InputStreamPtr&, TestMotivePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::HomingMotive> HomingMotivePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::HomingMotive> HomingMotivePrx;

void __read(::IceInternal::BasicStream*, HomingMotivePrx&);
void __patch__HomingMotivePtr(void*, ::Ice::ObjectPtr&);

void ice_writeHomingMotivePrx(const ::Ice::OutputStreamPtr&, const HomingMotivePrx&);
void ice_readHomingMotivePrx(const ::Ice::InputStreamPtr&, HomingMotivePrx&);
void ice_writeHomingMotive(const ::Ice::OutputStreamPtr&, const HomingMotivePtr&);
void ice_readHomingMotive(const ::Ice::InputStreamPtr&, HomingMotivePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::ExploreMotive> ExploreMotivePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::ExploreMotive> ExploreMotivePrx;

void __read(::IceInternal::BasicStream*, ExploreMotivePrx&);
void __patch__ExploreMotivePtr(void*, ::Ice::ObjectPtr&);

void ice_writeExploreMotivePrx(const ::Ice::OutputStreamPtr&, const ExploreMotivePrx&);
void ice_readExploreMotivePrx(const ::Ice::InputStreamPtr&, ExploreMotivePrx&);
void ice_writeExploreMotive(const ::Ice::OutputStreamPtr&, const ExploreMotivePtr&);
void ice_readExploreMotive(const ::Ice::InputStreamPtr&, ExploreMotivePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::CategorizePlaceMotive> CategorizePlaceMotivePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::CategorizePlaceMotive> CategorizePlaceMotivePrx;

void __read(::IceInternal::BasicStream*, CategorizePlaceMotivePrx&);
void __patch__CategorizePlaceMotivePtr(void*, ::Ice::ObjectPtr&);

void ice_writeCategorizePlaceMotivePrx(const ::Ice::OutputStreamPtr&, const CategorizePlaceMotivePrx&);
void ice_readCategorizePlaceMotivePrx(const ::Ice::InputStreamPtr&, CategorizePlaceMotivePrx&);
void ice_writeCategorizePlaceMotive(const ::Ice::OutputStreamPtr&, const CategorizePlaceMotivePtr&);
void ice_readCategorizePlaceMotive(const ::Ice::InputStreamPtr&, CategorizePlaceMotivePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::CategorizeRoomMotive> CategorizeRoomMotivePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::CategorizeRoomMotive> CategorizeRoomMotivePrx;

void __read(::IceInternal::BasicStream*, CategorizeRoomMotivePrx&);
void __patch__CategorizeRoomMotivePtr(void*, ::Ice::ObjectPtr&);

void ice_writeCategorizeRoomMotivePrx(const ::Ice::OutputStreamPtr&, const CategorizeRoomMotivePrx&);
void ice_readCategorizeRoomMotivePrx(const ::Ice::InputStreamPtr&, CategorizeRoomMotivePrx&);
void ice_writeCategorizeRoomMotive(const ::Ice::OutputStreamPtr&, const CategorizeRoomMotivePtr&);
void ice_readCategorizeRoomMotive(const ::Ice::InputStreamPtr&, CategorizeRoomMotivePtr&);

typedef ::IceInternal::Handle< ::motivation::slice::PlanProxy> PlanProxyPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::motivation::slice::PlanProxy> PlanProxyPrx;

void __read(::IceInternal::BasicStream*, PlanProxyPrx&);
void __patch__PlanProxyPtr(void*, ::Ice::ObjectPtr&);

void ice_writePlanProxyPrx(const ::Ice::OutputStreamPtr&, const PlanProxyPrx&);
void ice_readPlanProxyPrx(const ::Ice::InputStreamPtr&, PlanProxyPrx&);
void ice_writePlanProxy(const ::Ice::OutputStreamPtr&, const PlanProxyPtr&);
void ice_readPlanProxy(const ::Ice::InputStreamPtr&, PlanProxyPtr&);

}

}

namespace motivation
{

namespace slice
{

enum MotiveStatus
{
    UNSURFACED,
    SURFACED,
    POSSIBLE,
    IMPOSSIBLE,
    ACTIVE,
    COMPLETED
};

void __write(::IceInternal::BasicStream*, MotiveStatus);
void __read(::IceInternal::BasicStream*, MotiveStatus&);

void ice_writeMotiveStatus(const ::Ice::OutputStreamPtr&, MotiveStatus);
void ice_readMotiveStatus(const ::Ice::InputStreamPtr&, MotiveStatus&);

}

}

namespace IceProxy
{

namespace motivation
{

namespace slice
{

class TestSource : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<TestSource> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestSource> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestSource*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TestSource*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Motive : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Motive> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Motive> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Motive*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Motive*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class TestMotive : virtual public ::IceProxy::motivation::slice::Motive
{
public:
    
    ::IceInternal::ProxyHandle<TestMotive> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TestMotive> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TestMotive*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TestMotive*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class HomingMotive : virtual public ::IceProxy::motivation::slice::Motive
{
public:
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<HomingMotive> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<HomingMotive*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<HomingMotive*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ExploreMotive : virtual public ::IceProxy::motivation::slice::Motive
{
public:
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<ExploreMotive> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExploreMotive*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ExploreMotive*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class CategorizePlaceMotive : virtual public ::IceProxy::motivation::slice::Motive
{
public:
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizePlaceMotive> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizePlaceMotive*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CategorizePlaceMotive*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class CategorizeRoomMotive : virtual public ::IceProxy::motivation::slice::Motive
{
public:
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<CategorizeRoomMotive> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CategorizeRoomMotive*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CategorizeRoomMotive*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class PlanProxy : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlanProxy> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlanProxy*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<PlanProxy*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace motivation
{

namespace slice
{

class TestSource : virtual public ::IceDelegate::Ice::Object
{
public:
};

class Motive : virtual public ::IceDelegate::Ice::Object
{
public:
};

class TestMotive : virtual public ::IceDelegate::motivation::slice::Motive
{
public:
};

class HomingMotive : virtual public ::IceDelegate::motivation::slice::Motive
{
public:
};

class ExploreMotive : virtual public ::IceDelegate::motivation::slice::Motive
{
public:
};

class CategorizePlaceMotive : virtual public ::IceDelegate::motivation::slice::Motive
{
public:
};

class CategorizeRoomMotive : virtual public ::IceDelegate::motivation::slice::Motive
{
public:
};

class PlanProxy : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

}

}

namespace IceDelegateM
{

namespace motivation
{

namespace slice
{

class TestSource : virtual public ::IceDelegate::motivation::slice::TestSource,
                   virtual public ::IceDelegateM::Ice::Object
{
public:
};

class Motive : virtual public ::IceDelegate::motivation::slice::Motive,
               virtual public ::IceDelegateM::Ice::Object
{
public:
};

class TestMotive : virtual public ::IceDelegate::motivation::slice::TestMotive,
                   virtual public ::IceDelegateM::motivation::slice::Motive
{
public:
};

class HomingMotive : virtual public ::IceDelegate::motivation::slice::HomingMotive,
                     virtual public ::IceDelegateM::motivation::slice::Motive
{
public:
};

class ExploreMotive : virtual public ::IceDelegate::motivation::slice::ExploreMotive,
                      virtual public ::IceDelegateM::motivation::slice::Motive
{
public:
};

class CategorizePlaceMotive : virtual public ::IceDelegate::motivation::slice::CategorizePlaceMotive,
                              virtual public ::IceDelegateM::motivation::slice::Motive
{
public:
};

class CategorizeRoomMotive : virtual public ::IceDelegate::motivation::slice::CategorizeRoomMotive,
                             virtual public ::IceDelegateM::motivation::slice::Motive
{
public:
};

class PlanProxy : virtual public ::IceDelegate::motivation::slice::PlanProxy,
                  virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

}

}

namespace IceDelegateD
{

namespace motivation
{

namespace slice
{

class TestSource : virtual public ::IceDelegate::motivation::slice::TestSource,
                   virtual public ::IceDelegateD::Ice::Object
{
public:
};

class Motive : virtual public ::IceDelegate::motivation::slice::Motive,
               virtual public ::IceDelegateD::Ice::Object
{
public:
};

class TestMotive : virtual public ::IceDelegate::motivation::slice::TestMotive,
                   virtual public ::IceDelegateD::motivation::slice::Motive
{
public:
};

class HomingMotive : virtual public ::IceDelegate::motivation::slice::HomingMotive,
                     virtual public ::IceDelegateD::motivation::slice::Motive
{
public:
};

class ExploreMotive : virtual public ::IceDelegate::motivation::slice::ExploreMotive,
                      virtual public ::IceDelegateD::motivation::slice::Motive
{
public:
};

class CategorizePlaceMotive : virtual public ::IceDelegate::motivation::slice::CategorizePlaceMotive,
                              virtual public ::IceDelegateD::motivation::slice::Motive
{
public:
};

class CategorizeRoomMotive : virtual public ::IceDelegate::motivation::slice::CategorizeRoomMotive,
                             virtual public ::IceDelegateD::motivation::slice::Motive
{
public:
};

class PlanProxy : virtual public ::IceDelegate::motivation::slice::PlanProxy,
                  virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

}

}

namespace motivation
{

namespace slice
{

class TestSource : virtual public ::Ice::Object
{
public:

    typedef TestSourcePrx ProxyType;
    typedef TestSourcePtr PointerType;
    
    TestSource() {}
    TestSource(const ::std::string&, const ::cast::cdl::CASTTime&);
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

    virtual ~TestSource() {}

    friend class TestSource__staticInit;

public:

    ::std::string name;

    ::cast::cdl::CASTTime time;
};

class TestSource__staticInit
{
public:

    ::motivation::slice::TestSource _init;
};

static ::motivation::slice::TestSource__staticInit _TestSource_init;

class Motive : virtual public ::Ice::Object
{
public:

    typedef MotivePrx ProxyType;
    typedef MotivePtr PointerType;
    
    Motive() {}
    Motive(const ::cast::cdl::CASTTime&, const ::cast::cdl::CASTTime&, const ::cast::cdl::WorkingMemoryAddress&, const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&, ::motivation::slice::MotiveStatus, ::Ice::Long, ::Ice::Float, ::Ice::Float, ::Ice::Double, ::Ice::Int, const ::std::string&);
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

    virtual ~Motive() {}

public:

    ::cast::cdl::CASTTime created;

    ::cast::cdl::CASTTime updated;

    ::cast::cdl::WorkingMemoryAddress referenceEntry;

    ::std::string correspondingUnion;

    ::cast::cdl::WorkingMemoryAddress thisEntry;

    ::motivation::slice::MotiveStatus status;

    ::Ice::Long tries;

    ::Ice::Float priority;

    ::Ice::Float costs;

    ::Ice::Double informationGain;

    ::Ice::Int rank;

    ::std::string goal;
};

class TestMotive : virtual public ::motivation::slice::Motive
{
public:

    typedef TestMotivePrx ProxyType;
    typedef TestMotivePtr PointerType;
    
    TestMotive() {}
    TestMotive(const ::cast::cdl::CASTTime&, const ::cast::cdl::CASTTime&, const ::cast::cdl::WorkingMemoryAddress&, const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&, ::motivation::slice::MotiveStatus, ::Ice::Long, ::Ice::Float, ::Ice::Float, ::Ice::Double, ::Ice::Int, const ::std::string&, const ::std::string&);
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

    virtual ~TestMotive() {}

public:

    ::std::string value;
};

class HomingMotive : virtual public ::motivation::slice::Motive
{
public:

    typedef HomingMotivePrx ProxyType;
    typedef HomingMotivePtr PointerType;
    
    HomingMotive() {}
    HomingMotive(const ::cast::cdl::CASTTime&, const ::cast::cdl::CASTTime&, const ::cast::cdl::WorkingMemoryAddress&, const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&, ::motivation::slice::MotiveStatus, ::Ice::Long, ::Ice::Float, ::Ice::Float, ::Ice::Double, ::Ice::Int, const ::std::string&, ::Ice::Long);
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

    virtual ~HomingMotive() {}

public:

    ::Ice::Long homePlaceID;
};

class ExploreMotive : virtual public ::motivation::slice::Motive
{
public:

    typedef ExploreMotivePrx ProxyType;
    typedef ExploreMotivePtr PointerType;
    
    ExploreMotive() {}
    ExploreMotive(const ::cast::cdl::CASTTime&, const ::cast::cdl::CASTTime&, const ::cast::cdl::WorkingMemoryAddress&, const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&, ::motivation::slice::MotiveStatus, ::Ice::Long, ::Ice::Float, ::Ice::Float, ::Ice::Double, ::Ice::Int, const ::std::string&, ::Ice::Long);
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

    virtual ~ExploreMotive() {}

public:

    ::Ice::Long placeID;
};

class CategorizePlaceMotive : virtual public ::motivation::slice::Motive
{
public:

    typedef CategorizePlaceMotivePrx ProxyType;
    typedef CategorizePlaceMotivePtr PointerType;
    
    CategorizePlaceMotive() {}
    CategorizePlaceMotive(const ::cast::cdl::CASTTime&, const ::cast::cdl::CASTTime&, const ::cast::cdl::WorkingMemoryAddress&, const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&, ::motivation::slice::MotiveStatus, ::Ice::Long, ::Ice::Float, ::Ice::Float, ::Ice::Double, ::Ice::Int, const ::std::string&, ::Ice::Long);
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

    virtual ~CategorizePlaceMotive() {}

public:

    ::Ice::Long placeID;
};

class CategorizeRoomMotive : virtual public ::motivation::slice::Motive
{
public:

    typedef CategorizeRoomMotivePrx ProxyType;
    typedef CategorizeRoomMotivePtr PointerType;
    
    CategorizeRoomMotive() {}
    CategorizeRoomMotive(const ::cast::cdl::CASTTime&, const ::cast::cdl::CASTTime&, const ::cast::cdl::WorkingMemoryAddress&, const ::std::string&, const ::cast::cdl::WorkingMemoryAddress&, ::motivation::slice::MotiveStatus, ::Ice::Long, ::Ice::Float, ::Ice::Float, ::Ice::Double, ::Ice::Int, const ::std::string&);
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

    virtual ~CategorizeRoomMotive() {}
};

class PlanProxy : virtual public ::Ice::Object
{
public:

    typedef PlanProxyPrx ProxyType;
    typedef PlanProxyPtr PointerType;
    
    PlanProxy() {}
    explicit PlanProxy(const ::cast::cdl::WorkingMemoryAddress&);
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

    virtual ~PlanProxy() {}

public:

    ::cast::cdl::WorkingMemoryAddress planAddress;
};

}

}

#endif
