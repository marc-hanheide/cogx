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

#include <VisionData.hpp>
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

::Ice::Object* IceInternal::upCast(::VisionData::VisualObject* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::VisionData::VisualObject* p) { return p; }

::Ice::Object* IceInternal::upCast(::VisionData::DetectionCommand* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::VisionData::DetectionCommand* p) { return p; }

void
VisionData::__read(::IceInternal::BasicStream* __is, ::VisionData::VisualObjectPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::VisionData::VisualObject;
        v->__copyFrom(proxy);
    }
}

void
VisionData::__read(::IceInternal::BasicStream* __is, ::VisionData::DetectionCommandPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::VisionData::DetectionCommand;
        v->__copyFrom(proxy);
    }
}

bool
VisionData::VisualObjectView::operator==(const VisualObjectView& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(boundingBox != __rhs.boundingBox)
    {
        return false;
    }
    if(detectionConfidence != __rhs.detectionConfidence)
    {
        return false;
    }
    if(camId != __rhs.camId)
    {
        return false;
    }
    return true;
}

bool
VisionData::VisualObjectView::operator<(const VisualObjectView& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(boundingBox < __rhs.boundingBox)
    {
        return true;
    }
    else if(__rhs.boundingBox < boundingBox)
    {
        return false;
    }
    if(detectionConfidence < __rhs.detectionConfidence)
    {
        return true;
    }
    else if(__rhs.detectionConfidence < detectionConfidence)
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
    return false;
}

void
VisionData::VisualObjectView::__write(::IceInternal::BasicStream* __os) const
{
    boundingBox.__write(__os);
    __os->write(detectionConfidence);
    __os->write(camId);
}

void
VisionData::VisualObjectView::__read(::IceInternal::BasicStream* __is)
{
    boundingBox.__read(__is);
    __is->read(detectionConfidence);
    __is->read(camId);
}

void
VisionData::__writeVisualObjectViewSeq(::IceInternal::BasicStream* __os, const ::VisionData::VisualObjectView* begin, const ::VisionData::VisualObjectView* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
VisionData::__readVisualObjectViewSeq(::IceInternal::BasicStream* __is, ::VisionData::VisualObjectViewSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->checkFixedSeq(sz, 44);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
    }
}

const ::std::string&
IceProxy::VisionData::VisualObject::ice_staticId()
{
    return ::VisionData::VisualObject::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::VisionData::VisualObject::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::VisionData::VisualObject);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::VisionData::VisualObject::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::VisionData::VisualObject);
}

::IceProxy::Ice::Object*
IceProxy::VisionData::VisualObject::__newInstance() const
{
    return new VisualObject;
}

const ::std::string&
IceProxy::VisionData::DetectionCommand::ice_staticId()
{
    return ::VisionData::DetectionCommand::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::VisionData::DetectionCommand::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::VisionData::DetectionCommand);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::VisionData::DetectionCommand::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::VisionData::DetectionCommand);
}

::IceProxy::Ice::Object*
IceProxy::VisionData::DetectionCommand::__newInstance() const
{
    return new DetectionCommand;
}

VisionData::VisualObject::VisualObject(const ::cogx::Math::Pose3& __ice_pose, ::Ice::Double __ice_detectionConfidence, const ::cogx::Math::Sphere3& __ice_boundingSphere, const ::cast::cdl::CASTTime& __ice_time, const ::VisionData::VisualObjectViewSeq& __ice_views, const ::std::string& __ice_label) :
    pose(__ice_pose),
    detectionConfidence(__ice_detectionConfidence),
    boundingSphere(__ice_boundingSphere),
    time(__ice_time),
    views(__ice_views),
    label(__ice_label)
{
}

::Ice::ObjectPtr
VisionData::VisualObject::ice_clone() const
{
    ::VisionData::VisualObjectPtr __p = new ::VisionData::VisualObject(*this);
    return __p;
}

static const ::std::string __VisionData__VisualObject_ids[2] =
{
    "::Ice::Object",
    "::VisionData::VisualObject"
};

bool
VisionData::VisualObject::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__VisionData__VisualObject_ids, __VisionData__VisualObject_ids + 2, _s);
}

::std::vector< ::std::string>
VisionData::VisualObject::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__VisionData__VisualObject_ids[0], &__VisionData__VisualObject_ids[2]);
}

const ::std::string&
VisionData::VisualObject::ice_id(const ::Ice::Current&) const
{
    return __VisionData__VisualObject_ids[1];
}

const ::std::string&
VisionData::VisualObject::ice_staticId()
{
    return __VisionData__VisualObject_ids[1];
}

void
VisionData::VisualObject::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    pose.__write(__os);
    __os->write(detectionConfidence);
    boundingSphere.__write(__os);
    time.__write(__os);
    if(views.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::VisionData::__writeVisualObjectViewSeq(__os, &views[0], &views[0] + views.size());
    }
    __os->write(label);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
VisionData::VisualObject::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    pose.__read(__is);
    __is->read(detectionConfidence);
    boundingSphere.__read(__is);
    time.__read(__is);
    ::VisionData::__readVisualObjectViewSeq(__is, views);
    __is->read(label);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
VisionData::VisualObject::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type VisionData::VisualObject was not generated with stream support";
    throw ex;
}

void
VisionData::VisualObject::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type VisionData::VisualObject was not generated with stream support";
    throw ex;
}

class __F__VisionData__VisualObject : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::VisionData::VisualObject::ice_staticId());
        return new ::VisionData::VisualObject;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__VisionData__VisualObject_Ptr = new __F__VisionData__VisualObject;

const ::Ice::ObjectFactoryPtr&
VisionData::VisualObject::ice_factory()
{
    return __F__VisionData__VisualObject_Ptr;
}

class __F__VisionData__VisualObject__Init
{
public:

    __F__VisionData__VisualObject__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::VisionData::VisualObject::ice_staticId(), ::VisionData::VisualObject::ice_factory());
    }

    ~__F__VisionData__VisualObject__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::VisionData::VisualObject::ice_staticId());
    }
};

static __F__VisionData__VisualObject__Init __F__VisionData__VisualObject__i;

#ifdef __APPLE__
extern "C" { void __F__VisionData__VisualObject__initializer() {} }
#endif

void 
VisionData::__patch__VisualObjectPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::VisionData::VisualObjectPtr* p = static_cast< ::VisionData::VisualObjectPtr*>(__addr);
    assert(p);
    *p = ::VisionData::VisualObjectPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::VisionData::VisualObject::ice_staticId(), v->ice_id());
    }
}

bool
VisionData::operator==(const ::VisionData::VisualObject& l, const ::VisionData::VisualObject& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
VisionData::operator<(const ::VisionData::VisualObject& l, const ::VisionData::VisualObject& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

VisionData::DetectionCommand::DetectionCommand(const ::VisionData::StringSeq& __ice_labels) :
    labels(__ice_labels)
{
}

::Ice::ObjectPtr
VisionData::DetectionCommand::ice_clone() const
{
    ::VisionData::DetectionCommandPtr __p = new ::VisionData::DetectionCommand(*this);
    return __p;
}

static const ::std::string __VisionData__DetectionCommand_ids[2] =
{
    "::Ice::Object",
    "::VisionData::DetectionCommand"
};

bool
VisionData::DetectionCommand::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__VisionData__DetectionCommand_ids, __VisionData__DetectionCommand_ids + 2, _s);
}

::std::vector< ::std::string>
VisionData::DetectionCommand::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__VisionData__DetectionCommand_ids[0], &__VisionData__DetectionCommand_ids[2]);
}

const ::std::string&
VisionData::DetectionCommand::ice_id(const ::Ice::Current&) const
{
    return __VisionData__DetectionCommand_ids[1];
}

const ::std::string&
VisionData::DetectionCommand::ice_staticId()
{
    return __VisionData__DetectionCommand_ids[1];
}

void
VisionData::DetectionCommand::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    if(labels.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&labels[0], &labels[0] + labels.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
VisionData::DetectionCommand::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(labels);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
VisionData::DetectionCommand::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type VisionData::DetectionCommand was not generated with stream support";
    throw ex;
}

void
VisionData::DetectionCommand::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type VisionData::DetectionCommand was not generated with stream support";
    throw ex;
}

class __F__VisionData__DetectionCommand : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::VisionData::DetectionCommand::ice_staticId());
        return new ::VisionData::DetectionCommand;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__VisionData__DetectionCommand_Ptr = new __F__VisionData__DetectionCommand;

const ::Ice::ObjectFactoryPtr&
VisionData::DetectionCommand::ice_factory()
{
    return __F__VisionData__DetectionCommand_Ptr;
}

class __F__VisionData__DetectionCommand__Init
{
public:

    __F__VisionData__DetectionCommand__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::VisionData::DetectionCommand::ice_staticId(), ::VisionData::DetectionCommand::ice_factory());
    }

    ~__F__VisionData__DetectionCommand__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::VisionData::DetectionCommand::ice_staticId());
    }
};

static __F__VisionData__DetectionCommand__Init __F__VisionData__DetectionCommand__i;

#ifdef __APPLE__
extern "C" { void __F__VisionData__DetectionCommand__initializer() {} }
#endif

void 
VisionData::__patch__DetectionCommandPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::VisionData::DetectionCommandPtr* p = static_cast< ::VisionData::DetectionCommandPtr*>(__addr);
    assert(p);
    *p = ::VisionData::DetectionCommandPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::VisionData::DetectionCommand::ice_staticId(), v->ice_id());
    }
}

bool
VisionData::operator==(const ::VisionData::DetectionCommand& l, const ::VisionData::DetectionCommand& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
VisionData::operator<(const ::VisionData::DetectionCommand& l, const ::VisionData::DetectionCommand& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
