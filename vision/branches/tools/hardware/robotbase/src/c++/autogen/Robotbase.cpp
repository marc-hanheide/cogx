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

#include <Robotbase.hpp>
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

static const ::std::string __Robotbase__OdometryPushClient__receiveOdometry_name = "receiveOdometry";

static const ::std::string __Robotbase__RobotbaseServer__pullOdometry_name = "pullOdometry";

static const ::std::string __Robotbase__RobotbaseServer__execMotionCommand_name = "execMotionCommand";

static const ::std::string __Robotbase__RobotbaseServer__registerOdometryPushClient_name = "registerOdometryPushClient";

::Ice::Object* IceInternal::upCast(::Robotbase::OdometryPushClient* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::Robotbase::OdometryPushClient* p) { return p; }

::Ice::Object* IceInternal::upCast(::Robotbase::RobotbaseServer* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::Robotbase::RobotbaseServer* p) { return p; }

void
Robotbase::__read(::IceInternal::BasicStream* __is, ::Robotbase::OdometryPushClientPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::Robotbase::OdometryPushClient;
        v->__copyFrom(proxy);
    }
}

void
Robotbase::__read(::IceInternal::BasicStream* __is, ::Robotbase::RobotbaseServerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::Robotbase::RobotbaseServer;
        v->__copyFrom(proxy);
    }
}

bool
Robotbase::Pose2d::operator==(const Pose2d& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(x != __rhs.x)
    {
        return false;
    }
    if(y != __rhs.y)
    {
        return false;
    }
    if(theta != __rhs.theta)
    {
        return false;
    }
    return true;
}

bool
Robotbase::Pose2d::operator<(const Pose2d& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(x < __rhs.x)
    {
        return true;
    }
    else if(__rhs.x < x)
    {
        return false;
    }
    if(y < __rhs.y)
    {
        return true;
    }
    else if(__rhs.y < y)
    {
        return false;
    }
    if(theta < __rhs.theta)
    {
        return true;
    }
    else if(__rhs.theta < theta)
    {
        return false;
    }
    return false;
}

void
Robotbase::Pose2d::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
    __os->write(theta);
}

void
Robotbase::Pose2d::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
    __is->read(theta);
}

void
Robotbase::__writePose2dOpt(::IceInternal::BasicStream* __os, const ::Robotbase::Pose2d* begin, const ::Robotbase::Pose2d* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
Robotbase::__readPose2dOpt(::IceInternal::BasicStream* __is, ::Robotbase::Pose2dOpt& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->checkFixedSeq(sz, 24);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
    }
}

bool
Robotbase::Odometry::operator==(const Odometry& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(time != __rhs.time)
    {
        return false;
    }
    if(odompose != __rhs.odompose)
    {
        return false;
    }
    if(speed != __rhs.speed)
    {
        return false;
    }
    if(encoder != __rhs.encoder)
    {
        return false;
    }
    return true;
}

bool
Robotbase::Odometry::operator<(const Odometry& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(time < __rhs.time)
    {
        return true;
    }
    else if(__rhs.time < time)
    {
        return false;
    }
    if(odompose < __rhs.odompose)
    {
        return true;
    }
    else if(__rhs.odompose < odompose)
    {
        return false;
    }
    if(speed < __rhs.speed)
    {
        return true;
    }
    else if(__rhs.speed < speed)
    {
        return false;
    }
    if(encoder < __rhs.encoder)
    {
        return true;
    }
    else if(__rhs.encoder < encoder)
    {
        return false;
    }
    return false;
}

void
Robotbase::Odometry::__write(::IceInternal::BasicStream* __os) const
{
    time.__write(__os);
    if(odompose.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::Robotbase::__writePose2dOpt(__os, &odompose[0], &odompose[0] + odompose.size());
    }
    if(speed.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&speed[0], &speed[0] + speed.size());
    }
    if(encoder.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&encoder[0], &encoder[0] + encoder.size());
    }
}

void
Robotbase::Odometry::__read(::IceInternal::BasicStream* __is)
{
    time.__read(__is);
    ::Robotbase::__readPose2dOpt(__is, odompose);
    __is->read(speed);
    __is->read(encoder);
}

bool
Robotbase::MotionCommand::operator==(const MotionCommand& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(speed != __rhs.speed)
    {
        return false;
    }
    if(rotspeed != __rhs.rotspeed)
    {
        return false;
    }
    return true;
}

bool
Robotbase::MotionCommand::operator<(const MotionCommand& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(speed < __rhs.speed)
    {
        return true;
    }
    else if(__rhs.speed < speed)
    {
        return false;
    }
    if(rotspeed < __rhs.rotspeed)
    {
        return true;
    }
    else if(__rhs.rotspeed < rotspeed)
    {
        return false;
    }
    return false;
}

void
Robotbase::MotionCommand::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(speed);
    __os->write(rotspeed);
}

void
Robotbase::MotionCommand::__read(::IceInternal::BasicStream* __is)
{
    __is->read(speed);
    __is->read(rotspeed);
}

void
IceProxy::Robotbase::OdometryPushClient::receiveOdometry(const ::Robotbase::Odometry& odom, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::Robotbase::OdometryPushClient* __del = dynamic_cast< ::IceDelegate::Robotbase::OdometryPushClient*>(__delBase.get());
            __del->receiveOdometry(odom, __ctx);
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
IceProxy::Robotbase::OdometryPushClient::ice_staticId()
{
    return ::Robotbase::OdometryPushClient::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::Robotbase::OdometryPushClient::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::Robotbase::OdometryPushClient);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::Robotbase::OdometryPushClient::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::Robotbase::OdometryPushClient);
}

::IceProxy::Ice::Object*
IceProxy::Robotbase::OdometryPushClient::__newInstance() const
{
    return new OdometryPushClient;
}

::Robotbase::Odometry
IceProxy::Robotbase::RobotbaseServer::pullOdometry(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Robotbase__RobotbaseServer__pullOdometry_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Robotbase::RobotbaseServer* __del = dynamic_cast< ::IceDelegate::Robotbase::RobotbaseServer*>(__delBase.get());
            return __del->pullOdometry(__ctx);
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
IceProxy::Robotbase::RobotbaseServer::execMotionCommand(const ::Robotbase::MotionCommand& cmd, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::Robotbase::RobotbaseServer* __del = dynamic_cast< ::IceDelegate::Robotbase::RobotbaseServer*>(__delBase.get());
            __del->execMotionCommand(cmd, __ctx);
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
IceProxy::Robotbase::RobotbaseServer::registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::Robotbase::RobotbaseServer* __del = dynamic_cast< ::IceDelegate::Robotbase::RobotbaseServer*>(__delBase.get());
            __del->registerOdometryPushClient(client, interval, __ctx);
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
IceProxy::Robotbase::RobotbaseServer::ice_staticId()
{
    return ::Robotbase::RobotbaseServer::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::Robotbase::RobotbaseServer::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::Robotbase::RobotbaseServer);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::Robotbase::RobotbaseServer::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::Robotbase::RobotbaseServer);
}

::IceProxy::Ice::Object*
IceProxy::Robotbase::RobotbaseServer::__newInstance() const
{
    return new RobotbaseServer;
}

void
IceDelegateM::Robotbase::OdometryPushClient::receiveOdometry(const ::Robotbase::Odometry& odom, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Robotbase__OdometryPushClient__receiveOdometry_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        odom.__write(__os);
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

::Robotbase::Odometry
IceDelegateM::Robotbase::RobotbaseServer::pullOdometry(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Robotbase__RobotbaseServer__pullOdometry_name, ::Ice::Normal, __context);
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
        ::Robotbase::Odometry __ret;
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
IceDelegateM::Robotbase::RobotbaseServer::execMotionCommand(const ::Robotbase::MotionCommand& cmd, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Robotbase__RobotbaseServer__execMotionCommand_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        cmd.__write(__os);
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
IceDelegateM::Robotbase::RobotbaseServer::registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Robotbase__RobotbaseServer__registerOdometryPushClient_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPrx(::IceInternal::upCast(client.get())));
        __os->write(interval);
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
IceDelegateD::Robotbase::OdometryPushClient::receiveOdometry(const ::Robotbase::Odometry& odom, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Robotbase::Odometry& odom, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_odom(odom)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Robotbase::OdometryPushClient* servant = dynamic_cast< ::Robotbase::OdometryPushClient*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->receiveOdometry(_m_odom, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::Robotbase::Odometry& _m_odom;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Robotbase__OdometryPushClient__receiveOdometry_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(odom, __current);
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

::Robotbase::Odometry
IceDelegateD::Robotbase::RobotbaseServer::pullOdometry(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Robotbase::Odometry& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Robotbase::RobotbaseServer* servant = dynamic_cast< ::Robotbase::RobotbaseServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->pullOdometry(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Robotbase::Odometry& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Robotbase__RobotbaseServer__pullOdometry_name, ::Ice::Normal, __context);
    ::Robotbase::Odometry __result;
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
IceDelegateD::Robotbase::RobotbaseServer::execMotionCommand(const ::Robotbase::MotionCommand& cmd, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Robotbase::MotionCommand& cmd, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_cmd(cmd)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Robotbase::RobotbaseServer* servant = dynamic_cast< ::Robotbase::RobotbaseServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->execMotionCommand(_m_cmd, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::Robotbase::MotionCommand& _m_cmd;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Robotbase__RobotbaseServer__execMotionCommand_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(cmd, __current);
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
IceDelegateD::Robotbase::RobotbaseServer::registerOdometryPushClient(const ::Robotbase::OdometryPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Robotbase::OdometryPushClientPrx& client, ::Ice::Double interval, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_client(client),
            _m_interval(interval)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Robotbase::RobotbaseServer* servant = dynamic_cast< ::Robotbase::RobotbaseServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->registerOdometryPushClient(_m_client, _m_interval, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::Robotbase::OdometryPushClientPrx& _m_client;
        ::Ice::Double _m_interval;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Robotbase__RobotbaseServer__registerOdometryPushClient_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(client, interval, __current);
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

::Ice::ObjectPtr
Robotbase::OdometryPushClient::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __Robotbase__OdometryPushClient_ids[2] =
{
    "::Ice::Object",
    "::Robotbase::OdometryPushClient"
};

bool
Robotbase::OdometryPushClient::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__Robotbase__OdometryPushClient_ids, __Robotbase__OdometryPushClient_ids + 2, _s);
}

::std::vector< ::std::string>
Robotbase::OdometryPushClient::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__Robotbase__OdometryPushClient_ids[0], &__Robotbase__OdometryPushClient_ids[2]);
}

const ::std::string&
Robotbase::OdometryPushClient::ice_id(const ::Ice::Current&) const
{
    return __Robotbase__OdometryPushClient_ids[1];
}

const ::std::string&
Robotbase::OdometryPushClient::ice_staticId()
{
    return __Robotbase__OdometryPushClient_ids[1];
}

::Ice::DispatchStatus
Robotbase::OdometryPushClient::___receiveOdometry(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Robotbase::Odometry odom;
    odom.__read(__is);
    __is->endReadEncaps();
    receiveOdometry(odom, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __Robotbase__OdometryPushClient_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "receiveOdometry"
};

::Ice::DispatchStatus
Robotbase::OdometryPushClient::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__Robotbase__OdometryPushClient_all, __Robotbase__OdometryPushClient_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __Robotbase__OdometryPushClient_all)
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
            return ___receiveOdometry(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
Robotbase::OdometryPushClient::__write(::IceInternal::BasicStream* __os) const
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
Robotbase::OdometryPushClient::__read(::IceInternal::BasicStream* __is, bool __rid)
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
Robotbase::OdometryPushClient::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Robotbase::OdometryPushClient was not generated with stream support";
    throw ex;
}

void
Robotbase::OdometryPushClient::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Robotbase::OdometryPushClient was not generated with stream support";
    throw ex;
}

void 
Robotbase::__patch__OdometryPushClientPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::Robotbase::OdometryPushClientPtr* p = static_cast< ::Robotbase::OdometryPushClientPtr*>(__addr);
    assert(p);
    *p = ::Robotbase::OdometryPushClientPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::Robotbase::OdometryPushClient::ice_staticId(), v->ice_id());
    }
}

bool
Robotbase::operator==(const ::Robotbase::OdometryPushClient& l, const ::Robotbase::OdometryPushClient& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
Robotbase::operator<(const ::Robotbase::OdometryPushClient& l, const ::Robotbase::OdometryPushClient& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
Robotbase::RobotbaseServer::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __Robotbase__RobotbaseServer_ids[3] =
{
    "::Ice::Object",
    "::Robotbase::RobotbaseServer",
    "::cast::interfaces::CASTComponent"
};

bool
Robotbase::RobotbaseServer::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__Robotbase__RobotbaseServer_ids, __Robotbase__RobotbaseServer_ids + 3, _s);
}

::std::vector< ::std::string>
Robotbase::RobotbaseServer::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__Robotbase__RobotbaseServer_ids[0], &__Robotbase__RobotbaseServer_ids[3]);
}

const ::std::string&
Robotbase::RobotbaseServer::ice_id(const ::Ice::Current&) const
{
    return __Robotbase__RobotbaseServer_ids[1];
}

const ::std::string&
Robotbase::RobotbaseServer::ice_staticId()
{
    return __Robotbase__RobotbaseServer_ids[1];
}

::Ice::DispatchStatus
Robotbase::RobotbaseServer::___pullOdometry(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Robotbase::Odometry __ret = pullOdometry(__current);
    __ret.__write(__os);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Robotbase::RobotbaseServer::___execMotionCommand(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Robotbase::MotionCommand cmd;
    cmd.__read(__is);
    __is->endReadEncaps();
    execMotionCommand(cmd, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Robotbase::RobotbaseServer::___registerOdometryPushClient(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Robotbase::OdometryPushClientPrx client;
    ::Ice::Double interval;
    ::Robotbase::__read(__is, client);
    __is->read(interval);
    __is->endReadEncaps();
    registerOdometryPushClient(client, interval, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __Robotbase__RobotbaseServer_all[] =
{
    "beat",
    "configure",
    "execMotionCommand",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "pullOdometry",
    "registerOdometryPushClient",
    "run",
    "setComponentManager",
    "setID",
    "start",
    "stop"
};

::Ice::DispatchStatus
Robotbase::RobotbaseServer::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__Robotbase__RobotbaseServer_all, __Robotbase__RobotbaseServer_all + 15, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __Robotbase__RobotbaseServer_all)
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
            return ___execMotionCommand(in, current);
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
            return ___pullOdometry(in, current);
        }
        case 9:
        {
            return ___registerOdometryPushClient(in, current);
        }
        case 10:
        {
            return ___run(in, current);
        }
        case 11:
        {
            return ___setComponentManager(in, current);
        }
        case 12:
        {
            return ___setID(in, current);
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
Robotbase::RobotbaseServer::__write(::IceInternal::BasicStream* __os) const
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
Robotbase::RobotbaseServer::__read(::IceInternal::BasicStream* __is, bool __rid)
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
Robotbase::RobotbaseServer::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Robotbase::RobotbaseServer was not generated with stream support";
    throw ex;
}

void
Robotbase::RobotbaseServer::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Robotbase::RobotbaseServer was not generated with stream support";
    throw ex;
}

void 
Robotbase::__patch__RobotbaseServerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::Robotbase::RobotbaseServerPtr* p = static_cast< ::Robotbase::RobotbaseServerPtr*>(__addr);
    assert(p);
    *p = ::Robotbase::RobotbaseServerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::Robotbase::RobotbaseServer::ice_staticId(), v->ice_id());
    }
}

bool
Robotbase::operator==(const ::Robotbase::RobotbaseServer& l, const ::Robotbase::RobotbaseServer& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
Robotbase::operator<(const ::Robotbase::RobotbaseServer& l, const ::Robotbase::RobotbaseServer& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
