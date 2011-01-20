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

#include <RGBD.hpp>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <Ice/Stream.h>
#include <IceUtil/Iterator.h>
#include <IceUtil/ScopedArray.h>

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

static const ::std::string __RGBD__KinectPushClient__receiveKinectData_name = "receiveKinectData";

static const ::std::string __RGBD__RGBDPushServer__registerKinectPushClient_name = "registerKinectPushClient";

::Ice::Object* IceInternal::upCast(::RGBD::KinectPushClient* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RGBD::KinectPushClient* p) { return p; }

::Ice::Object* IceInternal::upCast(::RGBD::RGBDPushServer* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RGBD::RGBDPushServer* p) { return p; }

void
RGBD::__read(::IceInternal::BasicStream* __is, ::RGBD::KinectPushClientPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RGBD::KinectPushClient;
        v->__copyFrom(proxy);
    }
}

void
RGBD::ice_writeKinectPushClientPrx(const ::Ice::OutputStreamPtr& __outS, const ::RGBD::KinectPushClientPrx& v)
{
    __outS->writeProxy(v);
}

void
RGBD::ice_readKinectPushClientPrx(const ::Ice::InputStreamPtr& __inS, ::RGBD::KinectPushClientPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RGBD::KinectPushClient;
        v->__copyFrom(proxy);
    }
}

void
RGBD::ice_writeKinectPushClient(const ::Ice::OutputStreamPtr& __outS, const ::RGBD::KinectPushClientPtr& v)
{
    __outS->writeObject(v);
}

void
RGBD::ice_readKinectPushClient(const ::Ice::InputStreamPtr& __inS, ::RGBD::KinectPushClientPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::RGBD::__patch__KinectPushClientPtr, &__v);
    __inS->readObject(__cb);
}

void
RGBD::__read(::IceInternal::BasicStream* __is, ::RGBD::RGBDPushServerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RGBD::RGBDPushServer;
        v->__copyFrom(proxy);
    }
}

void
RGBD::ice_writeRGBDPushServerPrx(const ::Ice::OutputStreamPtr& __outS, const ::RGBD::RGBDPushServerPrx& v)
{
    __outS->writeProxy(v);
}

void
RGBD::ice_readRGBDPushServerPrx(const ::Ice::InputStreamPtr& __inS, ::RGBD::RGBDPushServerPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RGBD::RGBDPushServer;
        v->__copyFrom(proxy);
    }
}

void
RGBD::ice_writeRGBDPushServer(const ::Ice::OutputStreamPtr& __outS, const ::RGBD::RGBDPushServerPtr& v)
{
    __outS->writeObject(v);
}

void
RGBD::ice_readRGBDPushServer(const ::Ice::InputStreamPtr& __inS, ::RGBD::RGBDPushServerPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::RGBD::__patch__RGBDPushServerPtr, &__v);
    __inS->readObject(__cb);
}

bool
RGBD::KinectData::operator==(const KinectData& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(XRes != __rhs.XRes)
    {
        return false;
    }
    if(YRes != __rhs.YRes)
    {
        return false;
    }
    if(depth != __rhs.depth)
    {
        return false;
    }
    if(frameid != __rhs.frameid)
    {
        return false;
    }
    return true;
}

bool
RGBD::KinectData::operator<(const KinectData& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(XRes < __rhs.XRes)
    {
        return true;
    }
    else if(__rhs.XRes < XRes)
    {
        return false;
    }
    if(YRes < __rhs.YRes)
    {
        return true;
    }
    else if(__rhs.YRes < YRes)
    {
        return false;
    }
    if(depth < __rhs.depth)
    {
        return true;
    }
    else if(__rhs.depth < depth)
    {
        return false;
    }
    if(frameid < __rhs.frameid)
    {
        return true;
    }
    else if(__rhs.frameid < frameid)
    {
        return false;
    }
    return false;
}

void
RGBD::KinectData::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(XRes);
    __os->write(YRes);
    if(depth.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&depth[0], &depth[0] + depth.size());
    }
    __os->write(frameid);
}

void
RGBD::KinectData::__read(::IceInternal::BasicStream* __is)
{
    __is->read(XRes);
    __is->read(YRes);
    __is->read(depth);
    __is->read(frameid);
}

void
RGBD::KinectData::ice_write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeInt(XRes);
    __outS->writeInt(YRes);
    __outS->writeIntSeq(depth);
    __outS->writeInt(frameid);
}

void
RGBD::KinectData::ice_read(const ::Ice::InputStreamPtr& __inS)
{
    XRes = __inS->readInt();
    YRes = __inS->readInt();
    depth = __inS->readIntSeq();
    frameid = __inS->readInt();
}

void
RGBD::ice_writeKinectData(const ::Ice::OutputStreamPtr& __outS, const ::RGBD::KinectData& __v)
{
    __v.ice_write(__outS);
}

void
RGBD::ice_readKinectData(const ::Ice::InputStreamPtr& __inS, ::RGBD::KinectData& __v)
{
    __v.ice_read(__inS);
}

void
IceProxy::RGBD::KinectPushClient::receiveKinectData(const ::RGBD::KinectData& data, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::RGBD::KinectPushClient* __del = dynamic_cast< ::IceDelegate::RGBD::KinectPushClient*>(__delBase.get());
            __del->receiveKinectData(data, __ctx);
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
IceProxy::RGBD::KinectPushClient::ice_staticId()
{
    return ::RGBD::KinectPushClient::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RGBD::KinectPushClient::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RGBD::KinectPushClient);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RGBD::KinectPushClient::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RGBD::KinectPushClient);
}

::IceProxy::Ice::Object*
IceProxy::RGBD::KinectPushClient::__newInstance() const
{
    return new KinectPushClient;
}

void
IceProxy::RGBD::RGBDPushServer::registerKinectPushClient(const ::RGBD::KinectPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::RGBD::RGBDPushServer* __del = dynamic_cast< ::IceDelegate::RGBD::RGBDPushServer*>(__delBase.get());
            __del->registerKinectPushClient(client, interval, __ctx);
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
IceProxy::RGBD::RGBDPushServer::ice_staticId()
{
    return ::RGBD::RGBDPushServer::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RGBD::RGBDPushServer::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RGBD::RGBDPushServer);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RGBD::RGBDPushServer::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RGBD::RGBDPushServer);
}

::IceProxy::Ice::Object*
IceProxy::RGBD::RGBDPushServer::__newInstance() const
{
    return new RGBDPushServer;
}

void
IceDelegateM::RGBD::KinectPushClient::receiveKinectData(const ::RGBD::KinectData& data, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RGBD__KinectPushClient__receiveKinectData_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        data.__write(__os);
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
IceDelegateM::RGBD::RGBDPushServer::registerKinectPushClient(const ::RGBD::KinectPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RGBD__RGBDPushServer__registerKinectPushClient_name, ::Ice::Normal, __context);
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
IceDelegateD::RGBD::KinectPushClient::receiveKinectData(const ::RGBD::KinectData& data, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::RGBD::KinectData& data, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_data(data)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RGBD::KinectPushClient* servant = dynamic_cast< ::RGBD::KinectPushClient*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->receiveKinectData(_m_data, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::RGBD::KinectData& _m_data;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RGBD__KinectPushClient__receiveKinectData_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(data, __current);
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
IceDelegateD::RGBD::RGBDPushServer::registerKinectPushClient(const ::RGBD::KinectPushClientPrx& client, ::Ice::Double interval, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::RGBD::KinectPushClientPrx& client, ::Ice::Double interval, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_client(client),
            _m_interval(interval)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RGBD::RGBDPushServer* servant = dynamic_cast< ::RGBD::RGBDPushServer*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->registerKinectPushClient(_m_client, _m_interval, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::RGBD::KinectPushClientPrx& _m_client;
        ::Ice::Double _m_interval;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RGBD__RGBDPushServer__registerKinectPushClient_name, ::Ice::Normal, __context);
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
RGBD::KinectPushClient::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __RGBD__KinectPushClient_ids[2] =
{
    "::Ice::Object",
    "::RGBD::KinectPushClient"
};

bool
RGBD::KinectPushClient::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RGBD__KinectPushClient_ids, __RGBD__KinectPushClient_ids + 2, _s);
}

::std::vector< ::std::string>
RGBD::KinectPushClient::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RGBD__KinectPushClient_ids[0], &__RGBD__KinectPushClient_ids[2]);
}

const ::std::string&
RGBD::KinectPushClient::ice_id(const ::Ice::Current&) const
{
    return __RGBD__KinectPushClient_ids[1];
}

const ::std::string&
RGBD::KinectPushClient::ice_staticId()
{
    return __RGBD__KinectPushClient_ids[1];
}

::Ice::DispatchStatus
RGBD::KinectPushClient::___receiveKinectData(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::RGBD::KinectData data;
    data.__read(__is);
    __is->endReadEncaps();
    receiveKinectData(data, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __RGBD__KinectPushClient_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "receiveKinectData"
};

::Ice::DispatchStatus
RGBD::KinectPushClient::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__RGBD__KinectPushClient_all, __RGBD__KinectPushClient_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __RGBD__KinectPushClient_all)
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
            return ___receiveKinectData(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
RGBD::KinectPushClient::__write(::IceInternal::BasicStream* __os) const
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
RGBD::KinectPushClient::__read(::IceInternal::BasicStream* __is, bool __rid)
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
RGBD::KinectPushClient::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
RGBD::KinectPushClient::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

void 
RGBD::__patch__KinectPushClientPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RGBD::KinectPushClientPtr* p = static_cast< ::RGBD::KinectPushClientPtr*>(__addr);
    assert(p);
    *p = ::RGBD::KinectPushClientPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RGBD::KinectPushClient::ice_staticId(), v->ice_id());
    }
}

bool
RGBD::operator==(const ::RGBD::KinectPushClient& l, const ::RGBD::KinectPushClient& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RGBD::operator<(const ::RGBD::KinectPushClient& l, const ::RGBD::KinectPushClient& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
RGBD::RGBDPushServer::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __RGBD__RGBDPushServer_ids[3] =
{
    "::Ice::Object",
    "::RGBD::RGBDPushServer",
    "::cast::interfaces::CASTComponent"
};

bool
RGBD::RGBDPushServer::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RGBD__RGBDPushServer_ids, __RGBD__RGBDPushServer_ids + 3, _s);
}

::std::vector< ::std::string>
RGBD::RGBDPushServer::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RGBD__RGBDPushServer_ids[0], &__RGBD__RGBDPushServer_ids[3]);
}

const ::std::string&
RGBD::RGBDPushServer::ice_id(const ::Ice::Current&) const
{
    return __RGBD__RGBDPushServer_ids[1];
}

const ::std::string&
RGBD::RGBDPushServer::ice_staticId()
{
    return __RGBD__RGBDPushServer_ids[1];
}

::Ice::DispatchStatus
RGBD::RGBDPushServer::___registerKinectPushClient(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::RGBD::KinectPushClientPrx client;
    ::Ice::Double interval;
    ::RGBD::__read(__is, client);
    __is->read(interval);
    __is->endReadEncaps();
    registerKinectPushClient(client, interval, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __RGBD__RGBDPushServer_all[] =
{
    "beat",
    "configure",
    "destroy",
    "getID",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "registerKinectPushClient",
    "run",
    "setComponentManager",
    "setID",
    "setTimeServer",
    "start",
    "stop"
};

::Ice::DispatchStatus
RGBD::RGBDPushServer::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__RGBD__RGBDPushServer_all, __RGBD__RGBDPushServer_all + 15, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __RGBD__RGBDPushServer_all)
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
            return ___registerKinectPushClient(in, current);
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
RGBD::RGBDPushServer::__write(::IceInternal::BasicStream* __os) const
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
RGBD::RGBDPushServer::__read(::IceInternal::BasicStream* __is, bool __rid)
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
RGBD::RGBDPushServer::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
RGBD::RGBDPushServer::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

void 
RGBD::__patch__RGBDPushServerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RGBD::RGBDPushServerPtr* p = static_cast< ::RGBD::RGBDPushServerPtr*>(__addr);
    assert(p);
    *p = ::RGBD::RGBDPushServerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RGBD::RGBDPushServer::ice_staticId(), v->ice_id());
    }
}

bool
RGBD::operator==(const ::RGBD::RGBDPushServer& l, const ::RGBD::RGBDPushServer& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RGBD::operator<(const ::RGBD::RGBDPushServer& l, const ::RGBD::RGBDPushServer& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
