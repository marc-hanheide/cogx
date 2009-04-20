// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `Video.ice'

#include <Video.hpp>
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

static const ::std::string __Video__VideoInterface__getNumCameras_name = "getNumCameras";

static const ::std::string __Video__VideoInterface__getImageSize_name = "getImageSize";

static const ::std::string __Video__VideoInterface__getFramerateMilliSeconds_name = "getFramerateMilliSeconds";

static const ::std::string __Video__VideoInterface__getImage_name = "getImage";

static const ::std::string __Video__VideoInterface__getImages_name = "getImages";

::Ice::Object* IceInternal::upCast(::Video::VideoInterface* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::Video::VideoInterface* p) { return p; }

void
Video::__read(::IceInternal::BasicStream* __is, ::Video::VideoInterfacePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::Video::VideoInterface;
        v->__copyFrom(proxy);
    }
}

bool
Video::CameraParameters::operator==(const CameraParameters& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(width != __rhs.width)
    {
        return false;
    }
    if(height != __rhs.height)
    {
        return false;
    }
    if(fx != __rhs.fx)
    {
        return false;
    }
    if(fy != __rhs.fy)
    {
        return false;
    }
    if(cx != __rhs.cx)
    {
        return false;
    }
    if(cy != __rhs.cy)
    {
        return false;
    }
    if(k1 != __rhs.k1)
    {
        return false;
    }
    if(k2 != __rhs.k2)
    {
        return false;
    }
    if(k3 != __rhs.k3)
    {
        return false;
    }
    if(p1 != __rhs.p1)
    {
        return false;
    }
    if(p2 != __rhs.p2)
    {
        return false;
    }
    if(pose != __rhs.pose)
    {
        return false;
    }
    return true;
}

bool
Video::CameraParameters::operator<(const CameraParameters& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(width < __rhs.width)
    {
        return true;
    }
    else if(__rhs.width < width)
    {
        return false;
    }
    if(height < __rhs.height)
    {
        return true;
    }
    else if(__rhs.height < height)
    {
        return false;
    }
    if(fx < __rhs.fx)
    {
        return true;
    }
    else if(__rhs.fx < fx)
    {
        return false;
    }
    if(fy < __rhs.fy)
    {
        return true;
    }
    else if(__rhs.fy < fy)
    {
        return false;
    }
    if(cx < __rhs.cx)
    {
        return true;
    }
    else if(__rhs.cx < cx)
    {
        return false;
    }
    if(cy < __rhs.cy)
    {
        return true;
    }
    else if(__rhs.cy < cy)
    {
        return false;
    }
    if(k1 < __rhs.k1)
    {
        return true;
    }
    else if(__rhs.k1 < k1)
    {
        return false;
    }
    if(k2 < __rhs.k2)
    {
        return true;
    }
    else if(__rhs.k2 < k2)
    {
        return false;
    }
    if(k3 < __rhs.k3)
    {
        return true;
    }
    else if(__rhs.k3 < k3)
    {
        return false;
    }
    if(p1 < __rhs.p1)
    {
        return true;
    }
    else if(__rhs.p1 < p1)
    {
        return false;
    }
    if(p2 < __rhs.p2)
    {
        return true;
    }
    else if(__rhs.p2 < p2)
    {
        return false;
    }
    if(pose < __rhs.pose)
    {
        return true;
    }
    else if(__rhs.pose < pose)
    {
        return false;
    }
    return false;
}

void
Video::CameraParameters::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(width);
    __os->write(height);
    __os->write(fx);
    __os->write(fy);
    __os->write(cx);
    __os->write(cy);
    __os->write(k1);
    __os->write(k2);
    __os->write(k3);
    __os->write(p1);
    __os->write(p2);
    pose.__write(__os);
}

void
Video::CameraParameters::__read(::IceInternal::BasicStream* __is)
{
    __is->read(width);
    __is->read(height);
    __is->read(fx);
    __is->read(fy);
    __is->read(cx);
    __is->read(cy);
    __is->read(k1);
    __is->read(k2);
    __is->read(k3);
    __is->read(p1);
    __is->read(p2);
    pose.__read(__is);
}

bool
Video::Image::operator==(const Image& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(width != __rhs.width)
    {
        return false;
    }
    if(height != __rhs.height)
    {
        return false;
    }
    if(data != __rhs.data)
    {
        return false;
    }
    if(time != __rhs.time)
    {
        return false;
    }
    if(camId != __rhs.camId)
    {
        return false;
    }
    if(camPars != __rhs.camPars)
    {
        return false;
    }
    return true;
}

bool
Video::Image::operator<(const Image& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(width < __rhs.width)
    {
        return true;
    }
    else if(__rhs.width < width)
    {
        return false;
    }
    if(height < __rhs.height)
    {
        return true;
    }
    else if(__rhs.height < height)
    {
        return false;
    }
    if(data < __rhs.data)
    {
        return true;
    }
    else if(__rhs.data < data)
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
    if(camId < __rhs.camId)
    {
        return true;
    }
    else if(__rhs.camId < camId)
    {
        return false;
    }
    if(camPars < __rhs.camPars)
    {
        return true;
    }
    else if(__rhs.camPars < camPars)
    {
        return false;
    }
    return false;
}

void
Video::Image::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(width);
    __os->write(height);
    if(data.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&data[0], &data[0] + data.size());
    }
    time.__write(__os);
    __os->write(camId);
    camPars.__write(__os);
}

void
Video::Image::__read(::IceInternal::BasicStream* __is)
{
    __is->read(width);
    __is->read(height);
    ::std::pair<const ::Ice::Byte*, const ::Ice::Byte*> ___data;
    __is->read(___data);
    ::std::vector< ::Ice::Byte>(___data.first, ___data.second).swap(data);
    time.__read(__is);
    __is->read(camId);
    camPars.__read(__is);
}

void
Video::__writeImageSeq(::IceInternal::BasicStream* __os, const ::Video::Image* begin, const ::Video::Image* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
Video::__readImageSeq(::IceInternal::BasicStream* __is, ::Video::ImageSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 205);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

::Ice::Int
IceProxy::Video::VideoInterface::getNumCameras(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Video__VideoInterface__getNumCameras_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Video::VideoInterface* __del = dynamic_cast< ::IceDelegate::Video::VideoInterface*>(__delBase.get());
            return __del->getNumCameras(__ctx);
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
IceProxy::Video::VideoInterface::getImageSize(::Ice::Int& width, ::Ice::Int& height, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Video__VideoInterface__getImageSize_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Video::VideoInterface* __del = dynamic_cast< ::IceDelegate::Video::VideoInterface*>(__delBase.get());
            __del->getImageSize(width, height, __ctx);
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

::Ice::Int
IceProxy::Video::VideoInterface::getFramerateMilliSeconds(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Video__VideoInterface__getFramerateMilliSeconds_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Video::VideoInterface* __del = dynamic_cast< ::IceDelegate::Video::VideoInterface*>(__delBase.get());
            return __del->getFramerateMilliSeconds(__ctx);
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
IceProxy::Video::VideoInterface::getImage(::Ice::Int camId, ::Video::Image& img, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Video__VideoInterface__getImage_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Video::VideoInterface* __del = dynamic_cast< ::IceDelegate::Video::VideoInterface*>(__delBase.get());
            __del->getImage(camId, img, __ctx);
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
IceProxy::Video::VideoInterface::getImages(::Video::ImageSeq& images, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__Video__VideoInterface__getImages_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::Video::VideoInterface* __del = dynamic_cast< ::IceDelegate::Video::VideoInterface*>(__delBase.get());
            __del->getImages(images, __ctx);
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
IceProxy::Video::VideoInterface::ice_staticId()
{
    return ::Video::VideoInterface::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::Video::VideoInterface::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::Video::VideoInterface);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::Video::VideoInterface::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::Video::VideoInterface);
}

::IceProxy::Ice::Object*
IceProxy::Video::VideoInterface::__newInstance() const
{
    return new VideoInterface;
}

::Ice::Int
IceDelegateM::Video::VideoInterface::getNumCameras(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Video__VideoInterface__getNumCameras_name, ::Ice::Normal, __context);
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

void
IceDelegateM::Video::VideoInterface::getImageSize(::Ice::Int& width, ::Ice::Int& height, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Video__VideoInterface__getImageSize_name, ::Ice::Normal, __context);
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(width);
        __is->read(height);
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::Ice::Int
IceDelegateM::Video::VideoInterface::getFramerateMilliSeconds(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Video__VideoInterface__getFramerateMilliSeconds_name, ::Ice::Normal, __context);
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

void
IceDelegateM::Video::VideoInterface::getImage(::Ice::Int camId, ::Video::Image& img, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Video__VideoInterface__getImage_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(camId);
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        img.__read(__is);
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::Video::VideoInterface::getImages(::Video::ImageSeq& images, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __Video__VideoInterface__getImages_name, ::Ice::Normal, __context);
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::Video::__readImageSeq(__is, images);
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::Ice::Int
IceDelegateD::Video::VideoInterface::getNumCameras(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Video::VideoInterface* servant = dynamic_cast< ::Video::VideoInterface*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getNumCameras(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Int& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Video__VideoInterface__getNumCameras_name, ::Ice::Normal, __context);
    ::Ice::Int __result;
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
IceDelegateD::Video::VideoInterface::getImageSize(::Ice::Int& width, ::Ice::Int& height, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int& width, ::Ice::Int& height, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_width(width),
            _m_height(height)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Video::VideoInterface* servant = dynamic_cast< ::Video::VideoInterface*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getImageSize(_m_width, _m_height, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Int& _m_width;
        ::Ice::Int& _m_height;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Video__VideoInterface__getImageSize_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(width, height, __current);
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

::Ice::Int
IceDelegateD::Video::VideoInterface::getFramerateMilliSeconds(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Video::VideoInterface* servant = dynamic_cast< ::Video::VideoInterface*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getFramerateMilliSeconds(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Int& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Video__VideoInterface__getFramerateMilliSeconds_name, ::Ice::Normal, __context);
    ::Ice::Int __result;
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
IceDelegateD::Video::VideoInterface::getImage(::Ice::Int camId, ::Video::Image& img, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int camId, ::Video::Image& img, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_camId(camId),
            _m_img(img)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Video::VideoInterface* servant = dynamic_cast< ::Video::VideoInterface*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getImage(_m_camId, _m_img, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Int _m_camId;
        ::Video::Image& _m_img;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Video__VideoInterface__getImage_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(camId, img, __current);
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
IceDelegateD::Video::VideoInterface::getImages(::Video::ImageSeq& images, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Video::ImageSeq& images, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_images(images)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::Video::VideoInterface* servant = dynamic_cast< ::Video::VideoInterface*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getImages(_m_images, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Video::ImageSeq& _m_images;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __Video__VideoInterface__getImages_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(images, __current);
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
Video::VideoInterface::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __Video__VideoInterface_ids[2] =
{
    "::Ice::Object",
    "::Video::VideoInterface"
};

bool
Video::VideoInterface::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__Video__VideoInterface_ids, __Video__VideoInterface_ids + 2, _s);
}

::std::vector< ::std::string>
Video::VideoInterface::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__Video__VideoInterface_ids[0], &__Video__VideoInterface_ids[2]);
}

const ::std::string&
Video::VideoInterface::ice_id(const ::Ice::Current&) const
{
    return __Video__VideoInterface_ids[1];
}

const ::std::string&
Video::VideoInterface::ice_staticId()
{
    return __Video__VideoInterface_ids[1];
}

::Ice::DispatchStatus
Video::VideoInterface::___getNumCameras(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Int __ret = getNumCameras(__current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Video::VideoInterface::___getImageSize(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Int width;
    ::Ice::Int height;
    getImageSize(width, height, __current);
    __os->write(width);
    __os->write(height);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Video::VideoInterface::___getFramerateMilliSeconds(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Int __ret = getFramerateMilliSeconds(__current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Video::VideoInterface::___getImage(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Ice::Int camId;
    __is->read(camId);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Video::Image img;
    getImage(camId, img, __current);
    img.__write(__os);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
Video::VideoInterface::___getImages(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Video::ImageSeq images;
    getImages(images, __current);
    if(images.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::Video::__writeImageSeq(__os, &images[0], &images[0] + images.size());
    }
    return ::Ice::DispatchOK;
}

static ::std::string __Video__VideoInterface_all[] =
{
    "getFramerateMilliSeconds",
    "getImage",
    "getImageSize",
    "getImages",
    "getNumCameras",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

::Ice::DispatchStatus
Video::VideoInterface::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__Video__VideoInterface_all, __Video__VideoInterface_all + 9, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __Video__VideoInterface_all)
    {
        case 0:
        {
            return ___getFramerateMilliSeconds(in, current);
        }
        case 1:
        {
            return ___getImage(in, current);
        }
        case 2:
        {
            return ___getImageSize(in, current);
        }
        case 3:
        {
            return ___getImages(in, current);
        }
        case 4:
        {
            return ___getNumCameras(in, current);
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
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
Video::VideoInterface::__write(::IceInternal::BasicStream* __os) const
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
Video::VideoInterface::__read(::IceInternal::BasicStream* __is, bool __rid)
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
Video::VideoInterface::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Video::VideoInterface was not generated with stream support";
    throw ex;
}

void
Video::VideoInterface::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Video::VideoInterface was not generated with stream support";
    throw ex;
}

void 
Video::__patch__VideoInterfacePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::Video::VideoInterfacePtr* p = static_cast< ::Video::VideoInterfacePtr*>(__addr);
    assert(p);
    *p = ::Video::VideoInterfacePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::Video::VideoInterface::ice_staticId(), v->ice_id());
    }
}

bool
Video::operator==(const ::Video::VideoInterface& l, const ::Video::VideoInterface& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
Video::operator<(const ::Video::VideoInterface& l, const ::Video::VideoInterface& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
