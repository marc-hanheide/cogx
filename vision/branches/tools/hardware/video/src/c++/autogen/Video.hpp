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

#ifndef ___home_tm_Programs_cogx_code_vision_branches_tools_hardware_video_src_c___autogen_Video_hpp__
#define ___home_tm_Programs_cogx_code_vision_branches_tools_hardware_video_src_c___autogen_Video_hpp__

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
#include <Math.hpp>
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

namespace Video
{

class VideoInterface;

}

}

namespace Video
{

class VideoInterface;
bool operator==(const VideoInterface&, const VideoInterface&);
bool operator<(const VideoInterface&, const VideoInterface&);

}

namespace IceInternal
{

::Ice::Object* upCast(::Video::VideoInterface*);
::IceProxy::Ice::Object* upCast(::IceProxy::Video::VideoInterface*);

}

namespace Video
{

typedef ::IceInternal::Handle< ::Video::VideoInterface> VideoInterfacePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::Video::VideoInterface> VideoInterfacePrx;

void __read(::IceInternal::BasicStream*, VideoInterfacePrx&);
void __patch__VideoInterfacePtr(void*, ::Ice::ObjectPtr&);

}

namespace Video
{

typedef ::std::vector< ::Ice::Byte> ByteSeq;

struct CameraParameters
{
    ::Ice::Int width;
    ::Ice::Int height;
    ::Ice::Double fx;
    ::Ice::Double fy;
    ::Ice::Double cx;
    ::Ice::Double cy;
    ::Ice::Double k1;
    ::Ice::Double k2;
    ::Ice::Double k3;
    ::Ice::Double p1;
    ::Ice::Double p2;
    ::cogx::Math::Pose3 pose;

    bool operator==(const CameraParameters&) const;
    bool operator<(const CameraParameters&) const;
    bool operator!=(const CameraParameters& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const CameraParameters& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const CameraParameters& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const CameraParameters& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Image
{
    ::Ice::Int width;
    ::Ice::Int height;
    ::Video::ByteSeq data;
    ::cast::cdl::CASTTime time;
    ::Ice::Int camId;
    ::Video::CameraParameters camPars;

    bool operator==(const Image&) const;
    bool operator<(const Image&) const;
    bool operator!=(const Image& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Image& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Image& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Image& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::Video::Image> ImageSeq;
void __writeImageSeq(::IceInternal::BasicStream*, const ::Video::Image*, const ::Video::Image*);
void __readImageSeq(::IceInternal::BasicStream*, ImageSeq&);

}

namespace IceProxy
{

namespace Video
{

class VideoInterface : virtual public ::IceProxy::Ice::Object
{
public:

    ::Ice::Int getNumCameras()
    {
        return getNumCameras(0);
    }
    ::Ice::Int getNumCameras(const ::Ice::Context& __ctx)
    {
        return getNumCameras(&__ctx);
    }
    
private:

    ::Ice::Int getNumCameras(const ::Ice::Context*);
    
public:

    void getImageSize(::Ice::Int& width, ::Ice::Int& height)
    {
        getImageSize(width, height, 0);
    }
    void getImageSize(::Ice::Int& width, ::Ice::Int& height, const ::Ice::Context& __ctx)
    {
        getImageSize(width, height, &__ctx);
    }
    
private:

    void getImageSize(::Ice::Int&, ::Ice::Int&, const ::Ice::Context*);
    
public:

    ::Ice::Int getFramerateMilliSeconds()
    {
        return getFramerateMilliSeconds(0);
    }
    ::Ice::Int getFramerateMilliSeconds(const ::Ice::Context& __ctx)
    {
        return getFramerateMilliSeconds(&__ctx);
    }
    
private:

    ::Ice::Int getFramerateMilliSeconds(const ::Ice::Context*);
    
public:

    void getImage(::Ice::Int camId, ::Video::Image& img)
    {
        getImage(camId, img, 0);
    }
    void getImage(::Ice::Int camId, ::Video::Image& img, const ::Ice::Context& __ctx)
    {
        getImage(camId, img, &__ctx);
    }
    
private:

    void getImage(::Ice::Int, ::Video::Image&, const ::Ice::Context*);
    
public:

    void getImages(::Video::ImageSeq& images)
    {
        getImages(images, 0);
    }
    void getImages(::Video::ImageSeq& images, const ::Ice::Context& __ctx)
    {
        getImages(images, &__ctx);
    }
    
private:

    void getImages(::Video::ImageSeq&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VideoInterface> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VideoInterface*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<VideoInterface*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace Video
{

class VideoInterface : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::Ice::Int getNumCameras(const ::Ice::Context*) = 0;

    virtual void getImageSize(::Ice::Int&, ::Ice::Int&, const ::Ice::Context*) = 0;

    virtual ::Ice::Int getFramerateMilliSeconds(const ::Ice::Context*) = 0;

    virtual void getImage(::Ice::Int, ::Video::Image&, const ::Ice::Context*) = 0;

    virtual void getImages(::Video::ImageSeq&, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace Video
{

class VideoInterface : virtual public ::IceDelegate::Video::VideoInterface,
                       virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::Ice::Int getNumCameras(const ::Ice::Context*);

    virtual void getImageSize(::Ice::Int&, ::Ice::Int&, const ::Ice::Context*);

    virtual ::Ice::Int getFramerateMilliSeconds(const ::Ice::Context*);

    virtual void getImage(::Ice::Int, ::Video::Image&, const ::Ice::Context*);

    virtual void getImages(::Video::ImageSeq&, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace Video
{

class VideoInterface : virtual public ::IceDelegate::Video::VideoInterface,
                       virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::Ice::Int getNumCameras(const ::Ice::Context*);

    virtual void getImageSize(::Ice::Int&, ::Ice::Int&, const ::Ice::Context*);

    virtual ::Ice::Int getFramerateMilliSeconds(const ::Ice::Context*);

    virtual void getImage(::Ice::Int, ::Video::Image&, const ::Ice::Context*);

    virtual void getImages(::Video::ImageSeq&, const ::Ice::Context*);
};

}

}

namespace Video
{

class VideoInterface : virtual public ::Ice::Object
{
public:

    typedef VideoInterfacePrx ProxyType;
    typedef VideoInterfacePtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::Ice::Int getNumCameras(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getNumCameras(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getImageSize(::Ice::Int&, ::Ice::Int&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getImageSize(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::Int getFramerateMilliSeconds(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getFramerateMilliSeconds(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getImage(::Ice::Int, ::Video::Image&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getImage(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getImages(::Video::ImageSeq&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getImages(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
