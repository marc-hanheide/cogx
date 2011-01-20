// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `RGBD.ice'

#ifndef ___home_alper_Projects_dora_y2_tools_hardware_rgbd_src_c___autogen_RGBD_hpp__
#define ___home_alper_Projects_dora_y2_tools_hardware_rgbd_src_c___autogen_RGBD_hpp__

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
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RGBD
{

class KinectPushClient;

class RGBDPushServer;

}

}

namespace RGBD
{

class KinectPushClient;
bool operator==(const KinectPushClient&, const KinectPushClient&);
bool operator<(const KinectPushClient&, const KinectPushClient&);

class RGBDPushServer;
bool operator==(const RGBDPushServer&, const RGBDPushServer&);
bool operator<(const RGBDPushServer&, const RGBDPushServer&);

}

namespace IceInternal
{

::Ice::Object* upCast(::RGBD::KinectPushClient*);
::IceProxy::Ice::Object* upCast(::IceProxy::RGBD::KinectPushClient*);

::Ice::Object* upCast(::RGBD::RGBDPushServer*);
::IceProxy::Ice::Object* upCast(::IceProxy::RGBD::RGBDPushServer*);

}

namespace RGBD
{

typedef ::IceInternal::Handle< ::RGBD::KinectPushClient> KinectPushClientPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RGBD::KinectPushClient> KinectPushClientPrx;

void __read(::IceInternal::BasicStream*, KinectPushClientPrx&);
void __patch__KinectPushClientPtr(void*, ::Ice::ObjectPtr&);

void ice_writeKinectPushClientPrx(const ::Ice::OutputStreamPtr&, const KinectPushClientPrx&);
void ice_readKinectPushClientPrx(const ::Ice::InputStreamPtr&, KinectPushClientPrx&);
void ice_writeKinectPushClient(const ::Ice::OutputStreamPtr&, const KinectPushClientPtr&);
void ice_readKinectPushClient(const ::Ice::InputStreamPtr&, KinectPushClientPtr&);

typedef ::IceInternal::Handle< ::RGBD::RGBDPushServer> RGBDPushServerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RGBD::RGBDPushServer> RGBDPushServerPrx;

void __read(::IceInternal::BasicStream*, RGBDPushServerPrx&);
void __patch__RGBDPushServerPtr(void*, ::Ice::ObjectPtr&);

void ice_writeRGBDPushServerPrx(const ::Ice::OutputStreamPtr&, const RGBDPushServerPrx&);
void ice_readRGBDPushServerPrx(const ::Ice::InputStreamPtr&, RGBDPushServerPrx&);
void ice_writeRGBDPushServer(const ::Ice::OutputStreamPtr&, const RGBDPushServerPtr&);
void ice_readRGBDPushServer(const ::Ice::InputStreamPtr&, RGBDPushServerPtr&);

}

namespace RGBD
{

typedef ::std::vector< ::Ice::Int> DepthData;

struct KinectData
{
    ::Ice::Int XRes;
    ::Ice::Int YRes;
    ::RGBD::DepthData depth;
    ::Ice::Int frameid;

    bool operator==(const KinectData&) const;
    bool operator<(const KinectData&) const;
    bool operator!=(const KinectData& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const KinectData& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const KinectData& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const KinectData& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);

    void ice_write(const ::Ice::OutputStreamPtr&) const;
    void ice_read(const ::Ice::InputStreamPtr&);
};

void ice_writeKinectData(const ::Ice::OutputStreamPtr&, const KinectData&);
void ice_readKinectData(const ::Ice::InputStreamPtr&, KinectData&);

}

namespace IceProxy
{

namespace RGBD
{

class KinectPushClient : virtual public ::IceProxy::Ice::Object
{
public:

    void receiveKinectData(const ::RGBD::KinectData& data)
    {
        receiveKinectData(data, 0);
    }
    void receiveKinectData(const ::RGBD::KinectData& data, const ::Ice::Context& __ctx)
    {
        receiveKinectData(data, &__ctx);
    }
    
private:

    void receiveKinectData(const ::RGBD::KinectData&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<KinectPushClient> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<KinectPushClient*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<KinectPushClient*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class RGBDPushServer : virtual public ::IceProxy::cast::interfaces::CASTComponent
{
public:

    void registerKinectPushClient(const ::RGBD::KinectPushClientPrx& client, ::Ice::Double interval)
    {
        registerKinectPushClient(client, interval, 0);
    }
    void registerKinectPushClient(const ::RGBD::KinectPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context& __ctx)
    {
        registerKinectPushClient(client, interval, &__ctx);
    }
    
private:

    void registerKinectPushClient(const ::RGBD::KinectPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RGBDPushServer> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RGBDPushServer*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<RGBDPushServer*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace IceDelegate
{

namespace RGBD
{

class KinectPushClient : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void receiveKinectData(const ::RGBD::KinectData&, const ::Ice::Context*) = 0;
};

class RGBDPushServer : virtual public ::IceDelegate::cast::interfaces::CASTComponent
{
public:

    virtual void registerKinectPushClient(const ::RGBD::KinectPushClientPrx&, ::Ice::Double, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace RGBD
{

class KinectPushClient : virtual public ::IceDelegate::RGBD::KinectPushClient,
                         virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void receiveKinectData(const ::RGBD::KinectData&, const ::Ice::Context*);
};

class RGBDPushServer : virtual public ::IceDelegate::RGBD::RGBDPushServer,
                       virtual public ::IceDelegateM::cast::interfaces::CASTComponent
{
public:

    virtual void registerKinectPushClient(const ::RGBD::KinectPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace RGBD
{

class KinectPushClient : virtual public ::IceDelegate::RGBD::KinectPushClient,
                         virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void receiveKinectData(const ::RGBD::KinectData&, const ::Ice::Context*);
};

class RGBDPushServer : virtual public ::IceDelegate::RGBD::RGBDPushServer,
                       virtual public ::IceDelegateD::cast::interfaces::CASTComponent
{
public:

    virtual void registerKinectPushClient(const ::RGBD::KinectPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
};

}

}

namespace RGBD
{

class KinectPushClient : virtual public ::Ice::Object
{
public:

    typedef KinectPushClientPrx ProxyType;
    typedef KinectPushClientPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void receiveKinectData(const ::RGBD::KinectData&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___receiveKinectData(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class RGBDPushServer : virtual public ::cast::interfaces::CASTComponent
{
public:

    typedef RGBDPushServerPrx ProxyType;
    typedef RGBDPushServerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void registerKinectPushClient(const ::RGBD::KinectPushClientPrx&, ::Ice::Double, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___registerKinectPushClient(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
