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

#include <Laser.hpp>
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

static const ::std::string __Laser__Scan2dPushClient__receiveScan2d_name = "receiveScan2d";

static const ::std::string __Laser__LaserServer__pullScan2d_name = "pullScan2d";

static const ::std::string __Laser__LaserServer__registerScan2dPushClient_name = "registerScan2dPushClient";

::Ice::Object* IceInternal::upCast(::Laser::Scan2dPushClient* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::Laser::Scan2dPushClient* p) { return p; }

::Ice::Object* IceInternal::upCast(::Laser::LaserServer* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::Laser::LaserServer* p) { return p; }

void
Laser::__read(::IceInternal::BasicStream* __is, ::Laser::Scan2dPushClientPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::Laser::Scan2dPushClient;
        v->__copyFrom(proxy);
    }
}

void
Laser::__read(::IceInternal::BasicStream* __is, ::Laser::LaserServerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::Laser::LaserServer;
        v->__copyFrom(proxy);
    }
}

bool
Laser::Scan2d::operator==(const Scan2d& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(time != __rhs.time)
    {
        return false;
    }
    if(ranges != __rhs.ranges)
    {
        return false;
    }
    if(startAngle != __rhs.startAngle)
    {
        return false;
    }
    if(angleStep != __rhs.angleStep)
    {
        return false;
    }
    if(maxRange != __rhs.maxRange)
    {
        return false;
    }
    if(minRange != __rhs.minRange)
    {
        return false;
    }
    if(rangeRes != __rhs.rangeRes)
    {
        return false;
    }
    return true;
}

bool
Laser::Scan2d::operator<(const Scan2d& __rhs) const
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
    if(ranges < __rhs.ranges)
    {
        return true;
    }
    else if(__rhs.ranges < ranges)
    {
        return false;
    }
    if(startAngle < __rhs.startAngle)
    {
        return true;
    }
    else if(__rhs.startAngle < startAngle)
    {
        return false;
    }
    if(angleStep < __rhs.angleStep)
    {
        return true;
    }
    else if(__rhs.angleStep < angleStep)
    {
        return false;
    }
    if(maxRange < __rhs.maxRange)
    {
        return true;
    }
    else if(__rhs.maxRange < maxRange)
    {
        return false;
    }
    if(minRange < __rhs.minRange)
    {
        return true;
    }
    else if(__rhs.minRange < minRange)
    {
        return false;
    }
    if(rangeRes < __rhs.rangeRes)
    {
        return true;
    }
    else if(__rhs.rangeRes < rangeRes)
    {
        return false;
    }
    return false;
}

void
Laser::Scan2d::__write(::IceInternal::BasicStream* __os) const
{
    time.__write(__os);
    if(ranges.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&ranges[0], &ranges[0] + ranges.size());
    }
    __os->write(startAngle);
    __os->write(angleStep);
    __os->write(maxRange);
    __os->write(minRange);
    __os->write(rangeRes);
}

void
Laser::Scan2d::__read(::IceInternal::BasicStream* __is)
{
    time.__read(__is);
    __is->read(ranges);
    __is->read(startAngle);
    __is->read(angleStep);
    __is->read(maxRange);
    __is->read(minRange);
    __is->read(rangeRes);
}

void
IceProxy::Laser::Scan2dPushClient::receiveScan2d(const ::Laser::Scan2d& scan, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::Laser::Scan2dPushClient* __del = dynamic_cast< ::IceDelegate::Laser::Scan2dPushClient*>(__delBase.get());
            __del->receiveScan2d(scan, __ctx);
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
IceProxy::Laser::Scan2dPushClient::ice_staticId()
{
    return ::Laser::Scan2dPushClient::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::Laser::Scan2dPushClient::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::Laser::Scan2dPushClient);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::Laser::Scan2dPushClient::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::Laser::Scan2dPushClient);
}

::IceProxy::Ice::Object*
IceProxy::Laser::Scan2dPushClient::__newInstance() const
{
    return new Scan2dPushClient;
}

::Laser::Scan2d
IceProxy::Laser::LaserServer::pullScan2d(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Laser__LaserServer__pullScan2d_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Laser::LaserServer* __del = dynamic_cast< ::IceDelegate::Laser::LaserServer*>(__delBase.get());
            return __del->pullScan2d(__ctx);
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
IceProxy::Laser::LaserServer::registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::Laser::LaserServer* __del = dynamic_cast< ::IceDelegate::Laser::LaserServer*>(__delBase.get());
            __del->registerScan2dPushClient(client, interval, __ctx);
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
IceProxy::Laser::LaserServer::ice_staticId()
{
    return ::Laser::LaserServer::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::Laser::LaserServer::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::Laser::LaserServer);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::Laser::LaserServer::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::Laser::LaserServer);
}

::IceProxy::Ice::Object*
IceProxy::Laser::LaserServer::__newInstance() const
{
    return new LaserServer;
}

void
IceDelegateM::Laser::Scan2dPushClient::receiveScan2d(const ::Laser::Scan2d& scan, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Laser__Scan2dPushClient__receiveScan2d_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        scan.__write(__os);
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

::Laser::Scan2d
IceDelegateM::Laser::LaserServer::pullScan2d(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Laser__LaserServer__pullScan2d_name, ::Ice::Normal, __context);
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
        ::Laser::Scan2d __ret;
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
IceDelegateM::Laser::LaserServer::registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Laser__LaserServer__registerScan2dPushClient_name, ::Ice::Normal, __context);
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
IceDelegateD::Laser::Scan2dPushClient::receiveScan2d(const ::Laser::Scan2d& scan, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Laser::Scan2d& scan, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_scan(scan)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Laser::Scan2dPushClient* servant = dynamic_cast< ::Laser::Scan2dPushClient*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->receiveScan2d(_m_scan, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::Laser::Scan2d& _m_scan;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Laser__Scan2dPushClient__receiveScan2d_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(scan, __current);
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

::Laser::Scan2d
IceDelegateD::Laser::LaserServer::pullScan2d(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Laser::Scan2d& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Laser::LaserServer* servant = dynamic_cast< ::Laser::LaserServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->pullScan2d(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Laser::Scan2d& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Laser__LaserServer__pullScan2d_name, ::Ice::Normal, __context);
    ::Laser::Scan2d __result;
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
IceDelegateD::Laser::LaserServer::registerScan2dPushClient(const ::Laser::Scan2dPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Laser::Scan2dPushClientPrx& client, ::Ice::Double interval, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_client(client),
            _m_interval(interval)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Laser::LaserServer* servant = dynamic_cast< ::Laser::LaserServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->registerScan2dPushClient(_m_client, _m_interval, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::Laser::Scan2dPushClientPrx& _m_client;
        ::Ice::Double _m_interval;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Laser__LaserServer__registerScan2dPushClient_name, ::Ice::Normal, __context);
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
Laser::Scan2dPushClient::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __Laser__Scan2dPushClient_ids[2] =
{
    "::Ice::Object",
    "::Laser::Scan2dPushClient"
};

bool
Laser::Scan2dPushClient::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__Laser__Scan2dPushClient_ids, __Laser__Scan2dPushClient_ids + 2, _s);
}

::std::vector< ::std::string>
Laser::Scan2dPushClient::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__Laser__Scan2dPushClient_ids[0], &__Laser__Scan2dPushClient_ids[2]);
}

const ::std::string&
Laser::Scan2dPushClient::ice_id(const ::Ice::Current&) const
{
    return __Laser__Scan2dPushClient_ids[1];
}

const ::std::string&
Laser::Scan2dPushClient::ice_staticId()
{
    return __Laser__Scan2dPushClient_ids[1];
}

::Ice::DispatchStatus
Laser::Scan2dPushClient::___receiveScan2d(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Laser::Scan2d scan;
    scan.__read(__is);
    __is->endReadEncaps();
    receiveScan2d(scan, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __Laser__Scan2dPushClient_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "receiveScan2d"
};

::Ice::DispatchStatus
Laser::Scan2dPushClient::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__Laser__Scan2dPushClient_all, __Laser__Scan2dPushClient_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __Laser__Scan2dPushClient_all)
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
            return ___receiveScan2d(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
Laser::Scan2dPushClient::__write(::IceInternal::BasicStream* __os) const
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
Laser::Scan2dPushClient::__read(::IceInternal::BasicStream* __is, bool __rid)
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
Laser::Scan2dPushClient::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Laser::Scan2dPushClient was not generated with stream support";
    throw ex;
}

void
Laser::Scan2dPushClient::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Laser::Scan2dPushClient was not generated with stream support";
    throw ex;
}

void 
Laser::__patch__Scan2dPushClientPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::Laser::Scan2dPushClientPtr* p = static_cast< ::Laser::Scan2dPushClientPtr*>(__addr);
    assert(p);
    *p = ::Laser::Scan2dPushClientPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::Laser::Scan2dPushClient::ice_staticId(), v->ice_id());
    }
}

bool
Laser::operator==(const ::Laser::Scan2dPushClient& l, const ::Laser::Scan2dPushClient& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
Laser::operator<(const ::Laser::Scan2dPushClient& l, const ::Laser::Scan2dPushClient& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
Laser::LaserServer::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __Laser__LaserServer_ids[3] =
{
    "::Ice::Object",
    "::Laser::LaserServer",
    "::cast::interfaces::CASTComponent"
};

bool
Laser::LaserServer::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__Laser__LaserServer_ids, __Laser__LaserServer_ids + 3, _s);
}

::std::vector< ::std::string>
Laser::LaserServer::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__Laser__LaserServer_ids[0], &__Laser__LaserServer_ids[3]);
}

const ::std::string&
Laser::LaserServer::ice_id(const ::Ice::Current&) const
{
    return __Laser__LaserServer_ids[1];
}

const ::std::string&
Laser::LaserServer::ice_staticId()
{
    return __Laser__LaserServer_ids[1];
}

::Ice::DispatchStatus
Laser::LaserServer::___pullScan2d(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Laser::Scan2d __ret = pullScan2d(__current);
    __ret.__write(__os);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Laser::LaserServer::___registerScan2dPushClient(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Laser::Scan2dPushClientPrx client;
    ::Ice::Double interval;
    ::Laser::__read(__is, client);
    __is->read(interval);
    __is->endReadEncaps();
    registerScan2dPushClient(client, interval, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __Laser__LaserServer_all[] =
{
    "beat",
    "configure",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "pullScan2d",
    "registerScan2dPushClient",
    "run",
    "setComponentManager",
    "setID",
    "start",
    "stop"
};

::Ice::DispatchStatus
Laser::LaserServer::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__Laser__LaserServer_all, __Laser__LaserServer_all + 14, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __Laser__LaserServer_all)
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
            return ___getID(in, current);
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
            return ___pullScan2d(in, current);
        }
        case 8:
        {
            return ___registerScan2dPushClient(in, current);
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
Laser::LaserServer::__write(::IceInternal::BasicStream* __os) const
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
Laser::LaserServer::__read(::IceInternal::BasicStream* __is, bool __rid)
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
Laser::LaserServer::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Laser::LaserServer was not generated with stream support";
    throw ex;
}

void
Laser::LaserServer::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Laser::LaserServer was not generated with stream support";
    throw ex;
}

void 
Laser::__patch__LaserServerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::Laser::LaserServerPtr* p = static_cast< ::Laser::LaserServerPtr*>(__addr);
    assert(p);
    *p = ::Laser::LaserServerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::Laser::LaserServer::ice_staticId(), v->ice_id());
    }
}

bool
Laser::operator==(const ::Laser::LaserServer& l, const ::Laser::LaserServer& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
Laser::operator<(const ::Laser::LaserServer& l, const ::Laser::LaserServer& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
