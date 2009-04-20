// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `Laser.ice'

#ifndef ___home_ari_cogx2_trunk_tools_hardware_laser_src_c___autogen_Laser_hpp__
#define ___home_ari_cogx2_trunk_tools_hardware_laser_src_c___autogen_Laser_hpp__

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
#include <CDL.hpp>
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

namespace Laser
{

class Scan2dPushClient;

class LaserServer;

}

}

namespace Laser
{

class Scan2dPushClient;
bool operator==(const Scan2dPushClient&, const Scan2dPushClient&);
bool operator<(const Scan2dPushClient&, const Scan2dPushClient&);

class LaserServer;
bool operator==(const LaserServer&, const LaserServer&);
bool operator<(const LaserServer&, const LaserServer&);

}

namespace IceInternal
{

::Ice::Object* upCast(::Laser::Scan2dPushClient*);
::IceProxy::Ice::Object* upCast(::IceProxy::Laser::Scan2dPushClient*);

::Ice::Object* upCast(::Laser::LaserServer*);
::IceProxy::Ice::Object* upCast(::IceProxy::Laser::LaserServer*);

}

namespace Laser
{

typedef ::IceInternal::Handle< ::Laser::Scan2dPushClient> Scan2dPushClientPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::Laser::Scan2dPushClient> Scan2dPushClientPrx;

void __read(::IceInternal::BasicStream*, Scan2dPushClientPrx&);
void __patch__Scan2dPushClientPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::Laser::LaserServer> LaserServerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::Laser::LaserServer> LaserServerPrx;

void __read(::IceInternal::BasicStream*, LaserServerPrx&);
void __patch__LaserServerPtr(void*, ::Ice::ObjectPtr&);

}

namespace Laser
{

typedef ::std::vector< ::Ice::Double> RangeSequence;

struct Scan2d
{
    ::cast::cdl::CASTTime time;
    ::Laser::RangeSequence ranges;
    ::Ice::Double startAngle;
    ::Ice::Double angleStep;
    ::Ice::Double maxRange;
    ::Ice::Double minRange;
    ::Ice::Double rangeRes;

    bool operator==(const Scan2d&) const;
    bool operator<(const Scan2d&) const;
    bool operator!=(const Scan2d& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Scan2d& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Scan2d& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Scan2d& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

}

namespace IceProxy
{

namespace Laser
{

class Scan2dPushClient : virtual public ::IceProxy::Ice::Object
{
public:

    void receiveScan2d(const ::Laser::Scan2d& scan)
    {
        receiveScan2d(scan, 0);
    }
    void receiveScan2d(const ::Laser::Scan2d& scan, const ::Ice::Context& __ctx)
    {
        receiveScan2d(scan, &__ctx);
    }
    
private:

    void receiveScan2d(const ::Laser::Scan2d&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Scan2dPushClient> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scan2dPushClient*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Scan2dPushClient*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class LaserServer : virtual public ::IceProxy::cast::interfaces::CASTComponent
{
public:

    ::Laser::Scan2d pullScan2d()
    {
        return pullScan2d(0);
    }
    ::Laser::Scan2d pullScan2d(const ::Ice::Context& __ctx)
    {
        return pullScan2d(&__ctx);
    }
    
private:

    ::Laser::Scan2d pullScan2d(const ::Ice::Context*);
    
public:

    void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx& client, ::Ice::Double interval)
    {
        registerScan2dPushClient(client, interval, 0);
    }
    void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context& __ctx)
    {
        registerScan2dPushClient(client, interval, &__ctx);
    }
    
private:

    void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<LaserServer> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<LaserServer> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<LaserServer*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<LaserServer*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace Laser
{

class Scan2dPushClient : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void receiveScan2d(const ::Laser::Scan2d&, const ::Ice::Context*) = 0;
};

class LaserServer : virtual public ::IceDelegate::cast::interfaces::CASTComponent
{
public:

    virtual ::Laser::Scan2d pullScan2d(const ::Ice::Context*) = 0;

    virtual void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx&, ::Ice::Double, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace Laser
{

class Scan2dPushClient : virtual public ::IceDelegate::Laser::Scan2dPushClient,
                         virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void receiveScan2d(const ::Laser::Scan2d&, const ::Ice::Context*);
};

class LaserServer : virtual public ::IceDelegate::Laser::LaserServer,
                    virtual public ::IceDelegateM::cast::interfaces::CASTComponent
{
public:

    virtual ::Laser::Scan2d pullScan2d(const ::Ice::Context*);

    virtual void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace Laser
{

class Scan2dPushClient : virtual public ::IceDelegate::Laser::Scan2dPushClient,
                         virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void receiveScan2d(const ::Laser::Scan2d&, const ::Ice::Context*);
};

class LaserServer : virtual public ::IceDelegate::Laser::LaserServer,
                    virtual public ::IceDelegateD::cast::interfaces::CASTComponent
{
public:

    virtual ::Laser::Scan2d pullScan2d(const ::Ice::Context*);

    virtual void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
};

}

}

namespace Laser
{

class Scan2dPushClient : virtual public ::Ice::Object
{
public:

    typedef Scan2dPushClientPrx ProxyType;
    typedef Scan2dPushClientPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void receiveScan2d(const ::Laser::Scan2d&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___receiveScan2d(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class LaserServer : virtual public ::cast::interfaces::CASTComponent
{
public:

    typedef LaserServerPrx ProxyType;
    typedef LaserServerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::Laser::Scan2d pullScan2d(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___pullScan2d(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx&, ::Ice::Double, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___registerScan2dPushClient(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
