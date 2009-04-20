// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `Robotbase.ice'

#ifndef ___home_ari_cogx2_trunk_tools_hardware_robotbase_src_c___autogen_Robotbase_hpp__
#define ___home_ari_cogx2_trunk_tools_hardware_robotbase_src_c___autogen_Robotbase_hpp__

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

namespace Robotbase
{

class OdometryPushClient;

class RobotbaseServer;

}

}

namespace Robotbase
{

class OdometryPushClient;
bool operator==(const OdometryPushClient&, const OdometryPushClient&);
bool operator<(const OdometryPushClient&, const OdometryPushClient&);

class RobotbaseServer;
bool operator==(const RobotbaseServer&, const RobotbaseServer&);
bool operator<(const RobotbaseServer&, const RobotbaseServer&);

}

namespace IceInternal
{

::Ice::Object* upCast(::Robotbase::OdometryPushClient*);
::IceProxy::Ice::Object* upCast(::IceProxy::Robotbase::OdometryPushClient*);

::Ice::Object* upCast(::Robotbase::RobotbaseServer*);
::IceProxy::Ice::Object* upCast(::IceProxy::Robotbase::RobotbaseServer*);

}

namespace Robotbase
{

typedef ::IceInternal::Handle< ::Robotbase::OdometryPushClient> OdometryPushClientPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::Robotbase::OdometryPushClient> OdometryPushClientPrx;

void __read(::IceInternal::BasicStream*, OdometryPushClientPrx&);
void __patch__OdometryPushClientPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::Robotbase::RobotbaseServer> RobotbaseServerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::Robotbase::RobotbaseServer> RobotbaseServerPrx;

void __read(::IceInternal::BasicStream*, RobotbaseServerPrx&);
void __patch__RobotbaseServerPtr(void*, ::Ice::ObjectPtr&);

}

namespace Robotbase
{

struct Pose2d
{
    ::Ice::Double x;
    ::Ice::Double y;
    ::Ice::Double theta;

    bool operator==(const Pose2d&) const;
    bool operator<(const Pose2d&) const;
    bool operator!=(const Pose2d& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Pose2d& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Pose2d& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Pose2d& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::Robotbase::Pose2d> Pose2dOpt;
void __writePose2dOpt(::IceInternal::BasicStream*, const ::Robotbase::Pose2d*, const ::Robotbase::Pose2d*);
void __readPose2dOpt(::IceInternal::BasicStream*, Pose2dOpt&);

typedef ::std::vector< ::Ice::Double> SpeedOpt;

typedef ::std::vector< ::Ice::Long> EncoderOpt;

struct Odometry
{
    ::cast::cdl::CASTTime time;
    ::Robotbase::Pose2dOpt odompose;
    ::Robotbase::SpeedOpt speed;
    ::Robotbase::EncoderOpt encoder;

    bool operator==(const Odometry&) const;
    bool operator<(const Odometry&) const;
    bool operator!=(const Odometry& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Odometry& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Odometry& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Odometry& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct MotionCommand
{
    ::Ice::Double speed;
    ::Ice::Double rotspeed;

    bool operator==(const MotionCommand&) const;
    bool operator<(const MotionCommand&) const;
    bool operator!=(const MotionCommand& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const MotionCommand& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const MotionCommand& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const MotionCommand& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

}

namespace IceProxy
{

namespace Robotbase
{

class OdometryPushClient : virtual public ::IceProxy::Ice::Object
{
public:

    void receiveOdometry(const ::Robotbase::Odometry& odom)
    {
        receiveOdometry(odom, 0);
    }
    void receiveOdometry(const ::Robotbase::Odometry& odom, const ::Ice::Context& __ctx)
    {
        receiveOdometry(odom, &__ctx);
    }
    
private:

    void receiveOdometry(const ::Robotbase::Odometry&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<OdometryPushClient> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<OdometryPushClient*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<OdometryPushClient*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class RobotbaseServer : virtual public ::IceProxy::cast::interfaces::CASTComponent
{
public:

    ::Robotbase::Odometry pullOdometry()
    {
        return pullOdometry(0);
    }
    ::Robotbase::Odometry pullOdometry(const ::Ice::Context& __ctx)
    {
        return pullOdometry(&__ctx);
    }
    
private:

    ::Robotbase::Odometry pullOdometry(const ::Ice::Context*);
    
public:

    void execMotionCommand(const ::Robotbase::MotionCommand& cmd)
    {
        execMotionCommand(cmd, 0);
    }
    void execMotionCommand(const ::Robotbase::MotionCommand& cmd, const ::Ice::Context& __ctx)
    {
        execMotionCommand(cmd, &__ctx);
    }
    
private:

    void execMotionCommand(const ::Robotbase::MotionCommand&, const ::Ice::Context*);
    
public:

    void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx& client, ::Ice::Double interval)
    {
        registerOdometryPushClient(client, interval, 0);
    }
    void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context& __ctx)
    {
        registerOdometryPushClient(client, interval, &__ctx);
    }
    
private:

    void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RobotbaseServer> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RobotbaseServer*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<RobotbaseServer*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace Robotbase
{

class OdometryPushClient : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void receiveOdometry(const ::Robotbase::Odometry&, const ::Ice::Context*) = 0;
};

class RobotbaseServer : virtual public ::IceDelegate::cast::interfaces::CASTComponent
{
public:

    virtual ::Robotbase::Odometry pullOdometry(const ::Ice::Context*) = 0;

    virtual void execMotionCommand(const ::Robotbase::MotionCommand&, const ::Ice::Context*) = 0;

    virtual void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx&, ::Ice::Double, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace Robotbase
{

class OdometryPushClient : virtual public ::IceDelegate::Robotbase::OdometryPushClient,
                           virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void receiveOdometry(const ::Robotbase::Odometry&, const ::Ice::Context*);
};

class RobotbaseServer : virtual public ::IceDelegate::Robotbase::RobotbaseServer,
                        virtual public ::IceDelegateM::cast::interfaces::CASTComponent
{
public:

    virtual ::Robotbase::Odometry pullOdometry(const ::Ice::Context*);

    virtual void execMotionCommand(const ::Robotbase::MotionCommand&, const ::Ice::Context*);

    virtual void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace Robotbase
{

class OdometryPushClient : virtual public ::IceDelegate::Robotbase::OdometryPushClient,
                           virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void receiveOdometry(const ::Robotbase::Odometry&, const ::Ice::Context*);
};

class RobotbaseServer : virtual public ::IceDelegate::Robotbase::RobotbaseServer,
                        virtual public ::IceDelegateD::cast::interfaces::CASTComponent
{
public:

    virtual ::Robotbase::Odometry pullOdometry(const ::Ice::Context*);

    virtual void execMotionCommand(const ::Robotbase::MotionCommand&, const ::Ice::Context*);

    virtual void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx&, ::Ice::Double, const ::Ice::Context*);
};

}

}

namespace Robotbase
{

class OdometryPushClient : virtual public ::Ice::Object
{
public:

    typedef OdometryPushClientPrx ProxyType;
    typedef OdometryPushClientPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void receiveOdometry(const ::Robotbase::Odometry&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___receiveOdometry(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class RobotbaseServer : virtual public ::cast::interfaces::CASTComponent
{
public:

    typedef RobotbaseServerPrx ProxyType;
    typedef RobotbaseServerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::Robotbase::Odometry pullOdometry(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___pullOdometry(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void execMotionCommand(const ::Robotbase::MotionCommand&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___execMotionCommand(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx&, ::Ice::Double, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___registerOdometryPushClient(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
