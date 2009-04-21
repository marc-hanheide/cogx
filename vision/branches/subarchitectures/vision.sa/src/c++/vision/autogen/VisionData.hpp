// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `VisionData.ice'

#ifndef ___home_tm_Work_03_CogX_subarchitectures_vision_sa_src_c___vision_autogen_VisionData_hpp__
#define ___home_tm_Work_03_CogX_subarchitectures_vision_sa_src_c___vision_autogen_VisionData_hpp__

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
#include <Math.hpp>
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

namespace VisionData
{

class VisualObject;

class DetectionCommand;

}

}

namespace VisionData
{

class VisualObject;
bool operator==(const VisualObject&, const VisualObject&);
bool operator<(const VisualObject&, const VisualObject&);

class DetectionCommand;
bool operator==(const DetectionCommand&, const DetectionCommand&);
bool operator<(const DetectionCommand&, const DetectionCommand&);

}

namespace IceInternal
{

::Ice::Object* upCast(::VisionData::VisualObject*);
::IceProxy::Ice::Object* upCast(::IceProxy::VisionData::VisualObject*);

::Ice::Object* upCast(::VisionData::DetectionCommand*);
::IceProxy::Ice::Object* upCast(::IceProxy::VisionData::DetectionCommand*);

}

namespace VisionData
{

typedef ::IceInternal::Handle< ::VisionData::VisualObject> VisualObjectPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::VisionData::VisualObject> VisualObjectPrx;

void __read(::IceInternal::BasicStream*, VisualObjectPrx&);
void __patch__VisualObjectPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::VisionData::DetectionCommand> DetectionCommandPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::VisionData::DetectionCommand> DetectionCommandPrx;

void __read(::IceInternal::BasicStream*, DetectionCommandPrx&);
void __patch__DetectionCommandPtr(void*, ::Ice::ObjectPtr&);

}

namespace VisionData
{

struct VisualObjectView
{
    ::cogx::Math::Rect2 boundingBox;
    ::Ice::Double detectionConfidence;
    ::Ice::Int camId;

    bool operator==(const VisualObjectView&) const;
    bool operator<(const VisualObjectView&) const;
    bool operator!=(const VisualObjectView& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const VisualObjectView& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const VisualObjectView& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const VisualObjectView& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::VisionData::VisualObjectView> VisualObjectViewSeq;
void __writeVisualObjectViewSeq(::IceInternal::BasicStream*, const ::VisionData::VisualObjectView*, const ::VisionData::VisualObjectView*);
void __readVisualObjectViewSeq(::IceInternal::BasicStream*, VisualObjectViewSeq&);

typedef ::std::vector< ::Ice::Int> IntSeq;

struct Vertex
{
    ::cogx::Math::Vector3 pos;
    ::cogx::Math::Vector3 normal;
    ::cogx::Math::Vector2 texCoord;

    bool operator==(const Vertex&) const;
    bool operator<(const Vertex&) const;
    bool operator!=(const Vertex& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Vertex& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Vertex& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Vertex& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::VisionData::Vertex> VertexSeq;
void __writeVertexSeq(::IceInternal::BasicStream*, const ::VisionData::Vertex*, const ::VisionData::Vertex*);
void __readVertexSeq(::IceInternal::BasicStream*, VertexSeq&);

struct Face
{
    ::VisionData::IntSeq vertices;

    bool operator==(const Face&) const;
    bool operator<(const Face&) const;
    bool operator!=(const Face& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Face& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Face& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Face& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::VisionData::Face> FaceSeq;
void __writeFaceSeq(::IceInternal::BasicStream*, const ::VisionData::Face*, const ::VisionData::Face*);
void __readFaceSeq(::IceInternal::BasicStream*, FaceSeq&);

struct ObjectGeometry
{
    ::VisionData::VertexSeq vertices;
    ::VisionData::FaceSeq faces;

    bool operator==(const ObjectGeometry&) const;
    bool operator<(const ObjectGeometry&) const;
    bool operator!=(const ObjectGeometry& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const ObjectGeometry& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const ObjectGeometry& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const ObjectGeometry& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::std::string> StringSeq;

}

namespace IceProxy
{

namespace VisionData
{

class VisualObject : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<VisualObject> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<VisualObject> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<VisualObject*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<VisualObject*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class DetectionCommand : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DetectionCommand> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DetectionCommand*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<DetectionCommand*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace VisionData
{

class VisualObject : virtual public ::IceDelegate::Ice::Object
{
public:
};

class DetectionCommand : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

}

namespace IceDelegateM
{

namespace VisionData
{

class VisualObject : virtual public ::IceDelegate::VisionData::VisualObject,
                     virtual public ::IceDelegateM::Ice::Object
{
public:
};

class DetectionCommand : virtual public ::IceDelegate::VisionData::DetectionCommand,
                         virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

}

namespace IceDelegateD
{

namespace VisionData
{

class VisualObject : virtual public ::IceDelegate::VisionData::VisualObject,
                     virtual public ::IceDelegateD::Ice::Object
{
public:
};

class DetectionCommand : virtual public ::IceDelegate::VisionData::DetectionCommand,
                         virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

}

namespace VisionData
{

class VisualObject : virtual public ::Ice::Object
{
public:

    typedef VisualObjectPrx ProxyType;
    typedef VisualObjectPtr PointerType;
    
    VisualObject() {}
    VisualObject(const ::cogx::Math::Pose3&, ::Ice::Double, const ::cogx::Math::Sphere3&, const ::cast::cdl::CASTTime&, const ::VisionData::VisualObjectViewSeq&, const ::std::string&);
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

    virtual ~VisualObject() {}

    friend class VisualObject__staticInit;

public:

    ::cogx::Math::Pose3 pose;

    ::Ice::Double detectionConfidence;

    ::cogx::Math::Sphere3 boundingSphere;

    ::cast::cdl::CASTTime time;

    ::VisionData::VisualObjectViewSeq views;

    ::std::string label;
};

class VisualObject__staticInit
{
public:

    ::VisionData::VisualObject _init;
};

static ::VisionData::VisualObject__staticInit _VisualObject_init;

class DetectionCommand : virtual public ::Ice::Object
{
public:

    typedef DetectionCommandPrx ProxyType;
    typedef DetectionCommandPtr PointerType;
    
    DetectionCommand() {}
    explicit DetectionCommand(const ::VisionData::StringSeq&);
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

    virtual ~DetectionCommand() {}

public:

    ::VisionData::StringSeq labels;
};

}

#endif
