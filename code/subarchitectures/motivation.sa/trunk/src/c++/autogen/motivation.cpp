// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `motivation.ice'

#include <motivation.hpp>
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
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

::Ice::Object* IceInternal::upCast(::motivation::slice::TestSource* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::TestSource* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::Motive* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::Motive* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::TestMotive* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::TestMotive* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::HomingMotive* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::HomingMotive* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::ExploreMotive* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::ExploreMotive* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::CategorizePlaceMotive* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::CategorizePlaceMotive* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::CategorizeRoomMotive* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::CategorizeRoomMotive* p) { return p; }

::Ice::Object* IceInternal::upCast(::motivation::slice::PlanProxy* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::motivation::slice::PlanProxy* p) { return p; }

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::TestSourcePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::TestSource;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeTestSourcePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::TestSourcePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readTestSourcePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::TestSourcePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::TestSource;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeTestSource(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::TestSourcePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readTestSource(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::TestSourcePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__TestSourcePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::MotivePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::Motive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeMotivePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::MotivePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readMotivePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::MotivePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::Motive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeMotive(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::MotivePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readMotive(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::MotivePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__MotivePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::TestMotivePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::TestMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeTestMotivePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::TestMotivePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readTestMotivePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::TestMotivePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::TestMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeTestMotive(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::TestMotivePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readTestMotive(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::TestMotivePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__TestMotivePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::HomingMotivePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::HomingMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeHomingMotivePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::HomingMotivePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readHomingMotivePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::HomingMotivePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::HomingMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeHomingMotive(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::HomingMotivePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readHomingMotive(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::HomingMotivePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__HomingMotivePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::ExploreMotivePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::ExploreMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeExploreMotivePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::ExploreMotivePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readExploreMotivePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::ExploreMotivePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::ExploreMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeExploreMotive(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::ExploreMotivePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readExploreMotive(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::ExploreMotivePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__ExploreMotivePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::CategorizePlaceMotivePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::CategorizePlaceMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeCategorizePlaceMotivePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::CategorizePlaceMotivePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readCategorizePlaceMotivePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::CategorizePlaceMotivePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::CategorizePlaceMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeCategorizePlaceMotive(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::CategorizePlaceMotivePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readCategorizePlaceMotive(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::CategorizePlaceMotivePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__CategorizePlaceMotivePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::CategorizeRoomMotivePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::CategorizeRoomMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeCategorizeRoomMotivePrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::CategorizeRoomMotivePrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readCategorizeRoomMotivePrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::CategorizeRoomMotivePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::CategorizeRoomMotive;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writeCategorizeRoomMotive(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::CategorizeRoomMotivePtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readCategorizeRoomMotive(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::CategorizeRoomMotivePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__CategorizeRoomMotivePtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::PlanProxyPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::PlanProxy;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writePlanProxyPrx(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::PlanProxyPrx& v)
{
    __outS->writeProxy(v);
}

void
motivation::slice::ice_readPlanProxyPrx(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::PlanProxyPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::motivation::slice::PlanProxy;
        v->__copyFrom(proxy);
    }
}

void
motivation::slice::ice_writePlanProxy(const ::Ice::OutputStreamPtr& __outS, const ::motivation::slice::PlanProxyPtr& v)
{
    __outS->writeObject(v);
}

void
motivation::slice::ice_readPlanProxy(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::PlanProxyPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::motivation::slice::__patch__PlanProxyPtr, &__v);
    __inS->readObject(__cb);
}

void
motivation::slice::__write(::IceInternal::BasicStream* __os, ::motivation::slice::MotiveStatus v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 6);
}

void
motivation::slice::__read(::IceInternal::BasicStream* __is, ::motivation::slice::MotiveStatus& v)
{
    ::Ice::Byte val;
    __is->read(val, 6);
    v = static_cast< ::motivation::slice::MotiveStatus>(val);
}

void
motivation::slice::ice_writeMotiveStatus(const ::Ice::OutputStreamPtr& __outS, ::motivation::slice::MotiveStatus v)
{
    if(static_cast<int>(v) >= 6)
    {
        throw ::Ice::MarshalException(__FILE__, __LINE__, "enumerator out of range");
    }
    __outS->writeByte(static_cast< ::Ice::Byte>(v));
}

void
motivation::slice::ice_readMotiveStatus(const ::Ice::InputStreamPtr& __inS, ::motivation::slice::MotiveStatus& v)
{
    ::Ice::Byte val = __inS->readByte();
    if(val > 6)
    {
        throw ::Ice::MarshalException(__FILE__, __LINE__, "enumerator out of range");
    }
    v = static_cast< ::motivation::slice::MotiveStatus>(val);
}

const ::std::string&
IceProxy::motivation::slice::TestSource::ice_staticId()
{
    return ::motivation::slice::TestSource::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::TestSource::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::TestSource);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::TestSource::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::TestSource);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::TestSource::__newInstance() const
{
    return new TestSource;
}

const ::std::string&
IceProxy::motivation::slice::Motive::ice_staticId()
{
    return ::motivation::slice::Motive::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::Motive::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::Motive);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::Motive::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::Motive);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::Motive::__newInstance() const
{
    return new Motive;
}

const ::std::string&
IceProxy::motivation::slice::TestMotive::ice_staticId()
{
    return ::motivation::slice::TestMotive::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::TestMotive::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::TestMotive);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::TestMotive::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::TestMotive);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::TestMotive::__newInstance() const
{
    return new TestMotive;
}

const ::std::string&
IceProxy::motivation::slice::HomingMotive::ice_staticId()
{
    return ::motivation::slice::HomingMotive::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::HomingMotive::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::HomingMotive);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::HomingMotive::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::HomingMotive);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::HomingMotive::__newInstance() const
{
    return new HomingMotive;
}

const ::std::string&
IceProxy::motivation::slice::ExploreMotive::ice_staticId()
{
    return ::motivation::slice::ExploreMotive::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::ExploreMotive::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::ExploreMotive);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::ExploreMotive::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::ExploreMotive);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::ExploreMotive::__newInstance() const
{
    return new ExploreMotive;
}

const ::std::string&
IceProxy::motivation::slice::CategorizePlaceMotive::ice_staticId()
{
    return ::motivation::slice::CategorizePlaceMotive::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::CategorizePlaceMotive::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::CategorizePlaceMotive);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::CategorizePlaceMotive::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::CategorizePlaceMotive);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::CategorizePlaceMotive::__newInstance() const
{
    return new CategorizePlaceMotive;
}

const ::std::string&
IceProxy::motivation::slice::CategorizeRoomMotive::ice_staticId()
{
    return ::motivation::slice::CategorizeRoomMotive::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::CategorizeRoomMotive::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::CategorizeRoomMotive);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::CategorizeRoomMotive::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::CategorizeRoomMotive);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::CategorizeRoomMotive::__newInstance() const
{
    return new CategorizeRoomMotive;
}

const ::std::string&
IceProxy::motivation::slice::PlanProxy::ice_staticId()
{
    return ::motivation::slice::PlanProxy::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::motivation::slice::PlanProxy::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::motivation::slice::PlanProxy);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::motivation::slice::PlanProxy::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::motivation::slice::PlanProxy);
}

::IceProxy::Ice::Object*
IceProxy::motivation::slice::PlanProxy::__newInstance() const
{
    return new PlanProxy;
}

motivation::slice::TestSource::TestSource(const ::std::string& __ice_name, const ::cast::cdl::CASTTime& __ice_time) :
    name(__ice_name),
    time(__ice_time)
{
}

::Ice::ObjectPtr
motivation::slice::TestSource::ice_clone() const
{
    ::motivation::slice::TestSourcePtr __p = new ::motivation::slice::TestSource(*this);
    return __p;
}

static const ::std::string __motivation__slice__TestSource_ids[2] =
{
    "::Ice::Object",
    "::motivation::slice::TestSource"
};

bool
motivation::slice::TestSource::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__TestSource_ids, __motivation__slice__TestSource_ids + 2, _s);
}

::std::vector< ::std::string>
motivation::slice::TestSource::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__TestSource_ids[0], &__motivation__slice__TestSource_ids[2]);
}

const ::std::string&
motivation::slice::TestSource::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__TestSource_ids[1];
}

const ::std::string&
motivation::slice::TestSource::ice_staticId()
{
    return __motivation__slice__TestSource_ids[1];
}

void
motivation::slice::TestSource::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(name);
    time.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
motivation::slice::TestSource::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(name);
    time.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
motivation::slice::TestSource::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(name);
    ::cast::cdl::ice_writeCASTTime(__outS, time);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
motivation::slice::TestSource::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    name = __inS->readString();
    ::cast::cdl::ice_readCASTTime(__inS, time);
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__motivation__slice__TestSource : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::TestSource::ice_staticId());
        return new ::motivation::slice::TestSource;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__TestSource_Ptr = new __F__motivation__slice__TestSource;

const ::Ice::ObjectFactoryPtr&
motivation::slice::TestSource::ice_factory()
{
    return __F__motivation__slice__TestSource_Ptr;
}

class __F__motivation__slice__TestSource__Init
{
public:

    __F__motivation__slice__TestSource__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::TestSource::ice_staticId(), ::motivation::slice::TestSource::ice_factory());
    }

    ~__F__motivation__slice__TestSource__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::TestSource::ice_staticId());
    }
};

static __F__motivation__slice__TestSource__Init __F__motivation__slice__TestSource__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__TestSource__initializer() {} }
#endif

void 
motivation::slice::__patch__TestSourcePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::TestSourcePtr* p = static_cast< ::motivation::slice::TestSourcePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::TestSourcePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::TestSource::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::TestSource& l, const ::motivation::slice::TestSource& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::TestSource& l, const ::motivation::slice::TestSource& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::Motive::Motive(const ::cast::cdl::CASTTime& __ice_created, const ::cast::cdl::CASTTime& __ice_updated, const ::cast::cdl::WorkingMemoryAddress& __ice_referenceEntry, const ::std::string& __ice_correspondingUnion, const ::cast::cdl::WorkingMemoryAddress& __ice_thisEntry, ::motivation::slice::MotiveStatus __ice_status, ::Ice::Long __ice_tries, ::Ice::Float __ice_priority, ::Ice::Float __ice_costs, ::Ice::Double __ice_informationGain, ::Ice::Int __ice_rank, const ::std::string& __ice_goal) :
    created(__ice_created),
    updated(__ice_updated),
    referenceEntry(__ice_referenceEntry),
    correspondingUnion(__ice_correspondingUnion),
    thisEntry(__ice_thisEntry),
    status(__ice_status),
    tries(__ice_tries),
    priority(__ice_priority),
    costs(__ice_costs),
    informationGain(__ice_informationGain),
    rank(__ice_rank),
    goal(__ice_goal)
{
}

::Ice::ObjectPtr
motivation::slice::Motive::ice_clone() const
{
    ::motivation::slice::MotivePtr __p = new ::motivation::slice::Motive(*this);
    return __p;
}

static const ::std::string __motivation__slice__Motive_ids[2] =
{
    "::Ice::Object",
    "::motivation::slice::Motive"
};

bool
motivation::slice::Motive::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__Motive_ids, __motivation__slice__Motive_ids + 2, _s);
}

::std::vector< ::std::string>
motivation::slice::Motive::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__Motive_ids[0], &__motivation__slice__Motive_ids[2]);
}

const ::std::string&
motivation::slice::Motive::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__Motive_ids[1];
}

const ::std::string&
motivation::slice::Motive::ice_staticId()
{
    return __motivation__slice__Motive_ids[1];
}

void
motivation::slice::Motive::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    created.__write(__os);
    updated.__write(__os);
    referenceEntry.__write(__os);
    __os->write(correspondingUnion);
    thisEntry.__write(__os);
    ::motivation::slice::__write(__os, status);
    __os->write(tries);
    __os->write(priority);
    __os->write(costs);
    __os->write(informationGain);
    __os->write(rank);
    __os->write(goal);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
motivation::slice::Motive::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    created.__read(__is);
    updated.__read(__is);
    referenceEntry.__read(__is);
    __is->read(correspondingUnion);
    thisEntry.__read(__is);
    ::motivation::slice::__read(__is, status);
    __is->read(tries);
    __is->read(priority);
    __is->read(costs);
    __is->read(informationGain);
    __is->read(rank);
    __is->read(goal);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
motivation::slice::Motive::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    ::cast::cdl::ice_writeCASTTime(__outS, created);
    ::cast::cdl::ice_writeCASTTime(__outS, updated);
    ::cast::cdl::ice_writeWorkingMemoryAddress(__outS, referenceEntry);
    __outS->writeString(correspondingUnion);
    ::cast::cdl::ice_writeWorkingMemoryAddress(__outS, thisEntry);
    ::motivation::slice::ice_writeMotiveStatus(__outS, status);
    __outS->writeLong(tries);
    __outS->writeFloat(priority);
    __outS->writeFloat(costs);
    __outS->writeDouble(informationGain);
    __outS->writeInt(rank);
    __outS->writeString(goal);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
motivation::slice::Motive::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    ::cast::cdl::ice_readCASTTime(__inS, created);
    ::cast::cdl::ice_readCASTTime(__inS, updated);
    ::cast::cdl::ice_readWorkingMemoryAddress(__inS, referenceEntry);
    correspondingUnion = __inS->readString();
    ::cast::cdl::ice_readWorkingMemoryAddress(__inS, thisEntry);
    ::motivation::slice::ice_readMotiveStatus(__inS, status);
    tries = __inS->readLong();
    priority = __inS->readFloat();
    costs = __inS->readFloat();
    informationGain = __inS->readDouble();
    rank = __inS->readInt();
    goal = __inS->readString();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__motivation__slice__Motive : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::Motive::ice_staticId());
        return new ::motivation::slice::Motive;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__Motive_Ptr = new __F__motivation__slice__Motive;

const ::Ice::ObjectFactoryPtr&
motivation::slice::Motive::ice_factory()
{
    return __F__motivation__slice__Motive_Ptr;
}

class __F__motivation__slice__Motive__Init
{
public:

    __F__motivation__slice__Motive__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::Motive::ice_staticId(), ::motivation::slice::Motive::ice_factory());
    }

    ~__F__motivation__slice__Motive__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::Motive::ice_staticId());
    }
};

static __F__motivation__slice__Motive__Init __F__motivation__slice__Motive__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__Motive__initializer() {} }
#endif

void 
motivation::slice::__patch__MotivePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::MotivePtr* p = static_cast< ::motivation::slice::MotivePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::MotivePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::Motive::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::Motive& l, const ::motivation::slice::Motive& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::Motive& l, const ::motivation::slice::Motive& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::TestMotive::TestMotive(const ::cast::cdl::CASTTime& __ice_created, const ::cast::cdl::CASTTime& __ice_updated, const ::cast::cdl::WorkingMemoryAddress& __ice_referenceEntry, const ::std::string& __ice_correspondingUnion, const ::cast::cdl::WorkingMemoryAddress& __ice_thisEntry, ::motivation::slice::MotiveStatus __ice_status, ::Ice::Long __ice_tries, ::Ice::Float __ice_priority, ::Ice::Float __ice_costs, ::Ice::Double __ice_informationGain, ::Ice::Int __ice_rank, const ::std::string& __ice_goal, const ::std::string& __ice_value) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#else
    ::motivation::slice::Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#endif
,
    value(__ice_value)
{
}

::Ice::ObjectPtr
motivation::slice::TestMotive::ice_clone() const
{
    ::motivation::slice::TestMotivePtr __p = new ::motivation::slice::TestMotive(*this);
    return __p;
}

static const ::std::string __motivation__slice__TestMotive_ids[3] =
{
    "::Ice::Object",
    "::motivation::slice::Motive",
    "::motivation::slice::TestMotive"
};

bool
motivation::slice::TestMotive::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__TestMotive_ids, __motivation__slice__TestMotive_ids + 3, _s);
}

::std::vector< ::std::string>
motivation::slice::TestMotive::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__TestMotive_ids[0], &__motivation__slice__TestMotive_ids[3]);
}

const ::std::string&
motivation::slice::TestMotive::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__TestMotive_ids[2];
}

const ::std::string&
motivation::slice::TestMotive::ice_staticId()
{
    return __motivation__slice__TestMotive_ids[2];
}

void
motivation::slice::TestMotive::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(value);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__os);
#else
    ::motivation::slice::Motive::__write(__os);
#endif
}

void
motivation::slice::TestMotive::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(value);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__is, true);
#else
    ::motivation::slice::Motive::__read(__is, true);
#endif
}

void
motivation::slice::TestMotive::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(value);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__outS);
#else
    ::motivation::slice::Motive::__write(__outS);
#endif
}

void
motivation::slice::TestMotive::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    value = __inS->readString();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__inS, true);
#else
    ::motivation::slice::Motive::__read(__inS, true);
#endif
}

class __F__motivation__slice__TestMotive : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::TestMotive::ice_staticId());
        return new ::motivation::slice::TestMotive;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__TestMotive_Ptr = new __F__motivation__slice__TestMotive;

const ::Ice::ObjectFactoryPtr&
motivation::slice::TestMotive::ice_factory()
{
    return __F__motivation__slice__TestMotive_Ptr;
}

class __F__motivation__slice__TestMotive__Init
{
public:

    __F__motivation__slice__TestMotive__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::TestMotive::ice_staticId(), ::motivation::slice::TestMotive::ice_factory());
    }

    ~__F__motivation__slice__TestMotive__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::TestMotive::ice_staticId());
    }
};

static __F__motivation__slice__TestMotive__Init __F__motivation__slice__TestMotive__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__TestMotive__initializer() {} }
#endif

void 
motivation::slice::__patch__TestMotivePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::TestMotivePtr* p = static_cast< ::motivation::slice::TestMotivePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::TestMotivePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::TestMotive::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::TestMotive& l, const ::motivation::slice::TestMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::TestMotive& l, const ::motivation::slice::TestMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::HomingMotive::HomingMotive(const ::cast::cdl::CASTTime& __ice_created, const ::cast::cdl::CASTTime& __ice_updated, const ::cast::cdl::WorkingMemoryAddress& __ice_referenceEntry, const ::std::string& __ice_correspondingUnion, const ::cast::cdl::WorkingMemoryAddress& __ice_thisEntry, ::motivation::slice::MotiveStatus __ice_status, ::Ice::Long __ice_tries, ::Ice::Float __ice_priority, ::Ice::Float __ice_costs, ::Ice::Double __ice_informationGain, ::Ice::Int __ice_rank, const ::std::string& __ice_goal, ::Ice::Long __ice_homePlaceID) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#else
    ::motivation::slice::Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#endif
,
    homePlaceID(__ice_homePlaceID)
{
}

::Ice::ObjectPtr
motivation::slice::HomingMotive::ice_clone() const
{
    ::motivation::slice::HomingMotivePtr __p = new ::motivation::slice::HomingMotive(*this);
    return __p;
}

static const ::std::string __motivation__slice__HomingMotive_ids[3] =
{
    "::Ice::Object",
    "::motivation::slice::HomingMotive",
    "::motivation::slice::Motive"
};

bool
motivation::slice::HomingMotive::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__HomingMotive_ids, __motivation__slice__HomingMotive_ids + 3, _s);
}

::std::vector< ::std::string>
motivation::slice::HomingMotive::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__HomingMotive_ids[0], &__motivation__slice__HomingMotive_ids[3]);
}

const ::std::string&
motivation::slice::HomingMotive::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__HomingMotive_ids[1];
}

const ::std::string&
motivation::slice::HomingMotive::ice_staticId()
{
    return __motivation__slice__HomingMotive_ids[1];
}

void
motivation::slice::HomingMotive::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(homePlaceID);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__os);
#else
    ::motivation::slice::Motive::__write(__os);
#endif
}

void
motivation::slice::HomingMotive::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(homePlaceID);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__is, true);
#else
    ::motivation::slice::Motive::__read(__is, true);
#endif
}

void
motivation::slice::HomingMotive::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeLong(homePlaceID);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__outS);
#else
    ::motivation::slice::Motive::__write(__outS);
#endif
}

void
motivation::slice::HomingMotive::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    homePlaceID = __inS->readLong();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__inS, true);
#else
    ::motivation::slice::Motive::__read(__inS, true);
#endif
}

class __F__motivation__slice__HomingMotive : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::HomingMotive::ice_staticId());
        return new ::motivation::slice::HomingMotive;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__HomingMotive_Ptr = new __F__motivation__slice__HomingMotive;

const ::Ice::ObjectFactoryPtr&
motivation::slice::HomingMotive::ice_factory()
{
    return __F__motivation__slice__HomingMotive_Ptr;
}

class __F__motivation__slice__HomingMotive__Init
{
public:

    __F__motivation__slice__HomingMotive__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::HomingMotive::ice_staticId(), ::motivation::slice::HomingMotive::ice_factory());
    }

    ~__F__motivation__slice__HomingMotive__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::HomingMotive::ice_staticId());
    }
};

static __F__motivation__slice__HomingMotive__Init __F__motivation__slice__HomingMotive__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__HomingMotive__initializer() {} }
#endif

void 
motivation::slice::__patch__HomingMotivePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::HomingMotivePtr* p = static_cast< ::motivation::slice::HomingMotivePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::HomingMotivePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::HomingMotive::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::HomingMotive& l, const ::motivation::slice::HomingMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::HomingMotive& l, const ::motivation::slice::HomingMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::ExploreMotive::ExploreMotive(const ::cast::cdl::CASTTime& __ice_created, const ::cast::cdl::CASTTime& __ice_updated, const ::cast::cdl::WorkingMemoryAddress& __ice_referenceEntry, const ::std::string& __ice_correspondingUnion, const ::cast::cdl::WorkingMemoryAddress& __ice_thisEntry, ::motivation::slice::MotiveStatus __ice_status, ::Ice::Long __ice_tries, ::Ice::Float __ice_priority, ::Ice::Float __ice_costs, ::Ice::Double __ice_informationGain, ::Ice::Int __ice_rank, const ::std::string& __ice_goal, ::Ice::Long __ice_placeID) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#else
    ::motivation::slice::Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#endif
,
    placeID(__ice_placeID)
{
}

::Ice::ObjectPtr
motivation::slice::ExploreMotive::ice_clone() const
{
    ::motivation::slice::ExploreMotivePtr __p = new ::motivation::slice::ExploreMotive(*this);
    return __p;
}

static const ::std::string __motivation__slice__ExploreMotive_ids[3] =
{
    "::Ice::Object",
    "::motivation::slice::ExploreMotive",
    "::motivation::slice::Motive"
};

bool
motivation::slice::ExploreMotive::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__ExploreMotive_ids, __motivation__slice__ExploreMotive_ids + 3, _s);
}

::std::vector< ::std::string>
motivation::slice::ExploreMotive::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__ExploreMotive_ids[0], &__motivation__slice__ExploreMotive_ids[3]);
}

const ::std::string&
motivation::slice::ExploreMotive::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__ExploreMotive_ids[1];
}

const ::std::string&
motivation::slice::ExploreMotive::ice_staticId()
{
    return __motivation__slice__ExploreMotive_ids[1];
}

void
motivation::slice::ExploreMotive::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(placeID);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__os);
#else
    ::motivation::slice::Motive::__write(__os);
#endif
}

void
motivation::slice::ExploreMotive::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(placeID);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__is, true);
#else
    ::motivation::slice::Motive::__read(__is, true);
#endif
}

void
motivation::slice::ExploreMotive::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeLong(placeID);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__outS);
#else
    ::motivation::slice::Motive::__write(__outS);
#endif
}

void
motivation::slice::ExploreMotive::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    placeID = __inS->readLong();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__inS, true);
#else
    ::motivation::slice::Motive::__read(__inS, true);
#endif
}

class __F__motivation__slice__ExploreMotive : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::ExploreMotive::ice_staticId());
        return new ::motivation::slice::ExploreMotive;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__ExploreMotive_Ptr = new __F__motivation__slice__ExploreMotive;

const ::Ice::ObjectFactoryPtr&
motivation::slice::ExploreMotive::ice_factory()
{
    return __F__motivation__slice__ExploreMotive_Ptr;
}

class __F__motivation__slice__ExploreMotive__Init
{
public:

    __F__motivation__slice__ExploreMotive__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::ExploreMotive::ice_staticId(), ::motivation::slice::ExploreMotive::ice_factory());
    }

    ~__F__motivation__slice__ExploreMotive__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::ExploreMotive::ice_staticId());
    }
};

static __F__motivation__slice__ExploreMotive__Init __F__motivation__slice__ExploreMotive__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__ExploreMotive__initializer() {} }
#endif

void 
motivation::slice::__patch__ExploreMotivePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::ExploreMotivePtr* p = static_cast< ::motivation::slice::ExploreMotivePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::ExploreMotivePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::ExploreMotive::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::ExploreMotive& l, const ::motivation::slice::ExploreMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::ExploreMotive& l, const ::motivation::slice::ExploreMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::CategorizePlaceMotive::CategorizePlaceMotive(const ::cast::cdl::CASTTime& __ice_created, const ::cast::cdl::CASTTime& __ice_updated, const ::cast::cdl::WorkingMemoryAddress& __ice_referenceEntry, const ::std::string& __ice_correspondingUnion, const ::cast::cdl::WorkingMemoryAddress& __ice_thisEntry, ::motivation::slice::MotiveStatus __ice_status, ::Ice::Long __ice_tries, ::Ice::Float __ice_priority, ::Ice::Float __ice_costs, ::Ice::Double __ice_informationGain, ::Ice::Int __ice_rank, const ::std::string& __ice_goal, ::Ice::Long __ice_placeID) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#else
    ::motivation::slice::Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#endif
,
    placeID(__ice_placeID)
{
}

::Ice::ObjectPtr
motivation::slice::CategorizePlaceMotive::ice_clone() const
{
    ::motivation::slice::CategorizePlaceMotivePtr __p = new ::motivation::slice::CategorizePlaceMotive(*this);
    return __p;
}

static const ::std::string __motivation__slice__CategorizePlaceMotive_ids[3] =
{
    "::Ice::Object",
    "::motivation::slice::CategorizePlaceMotive",
    "::motivation::slice::Motive"
};

bool
motivation::slice::CategorizePlaceMotive::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__CategorizePlaceMotive_ids, __motivation__slice__CategorizePlaceMotive_ids + 3, _s);
}

::std::vector< ::std::string>
motivation::slice::CategorizePlaceMotive::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__CategorizePlaceMotive_ids[0], &__motivation__slice__CategorizePlaceMotive_ids[3]);
}

const ::std::string&
motivation::slice::CategorizePlaceMotive::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__CategorizePlaceMotive_ids[1];
}

const ::std::string&
motivation::slice::CategorizePlaceMotive::ice_staticId()
{
    return __motivation__slice__CategorizePlaceMotive_ids[1];
}

void
motivation::slice::CategorizePlaceMotive::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(placeID);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__os);
#else
    ::motivation::slice::Motive::__write(__os);
#endif
}

void
motivation::slice::CategorizePlaceMotive::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(placeID);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__is, true);
#else
    ::motivation::slice::Motive::__read(__is, true);
#endif
}

void
motivation::slice::CategorizePlaceMotive::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeLong(placeID);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__outS);
#else
    ::motivation::slice::Motive::__write(__outS);
#endif
}

void
motivation::slice::CategorizePlaceMotive::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    placeID = __inS->readLong();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__inS, true);
#else
    ::motivation::slice::Motive::__read(__inS, true);
#endif
}

class __F__motivation__slice__CategorizePlaceMotive : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::CategorizePlaceMotive::ice_staticId());
        return new ::motivation::slice::CategorizePlaceMotive;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__CategorizePlaceMotive_Ptr = new __F__motivation__slice__CategorizePlaceMotive;

const ::Ice::ObjectFactoryPtr&
motivation::slice::CategorizePlaceMotive::ice_factory()
{
    return __F__motivation__slice__CategorizePlaceMotive_Ptr;
}

class __F__motivation__slice__CategorizePlaceMotive__Init
{
public:

    __F__motivation__slice__CategorizePlaceMotive__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::CategorizePlaceMotive::ice_staticId(), ::motivation::slice::CategorizePlaceMotive::ice_factory());
    }

    ~__F__motivation__slice__CategorizePlaceMotive__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::CategorizePlaceMotive::ice_staticId());
    }
};

static __F__motivation__slice__CategorizePlaceMotive__Init __F__motivation__slice__CategorizePlaceMotive__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__CategorizePlaceMotive__initializer() {} }
#endif

void 
motivation::slice::__patch__CategorizePlaceMotivePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::CategorizePlaceMotivePtr* p = static_cast< ::motivation::slice::CategorizePlaceMotivePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::CategorizePlaceMotivePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::CategorizePlaceMotive::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::CategorizePlaceMotive& l, const ::motivation::slice::CategorizePlaceMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::CategorizePlaceMotive& l, const ::motivation::slice::CategorizePlaceMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::CategorizeRoomMotive::CategorizeRoomMotive(const ::cast::cdl::CASTTime& __ice_created, const ::cast::cdl::CASTTime& __ice_updated, const ::cast::cdl::WorkingMemoryAddress& __ice_referenceEntry, const ::std::string& __ice_correspondingUnion, const ::cast::cdl::WorkingMemoryAddress& __ice_thisEntry, ::motivation::slice::MotiveStatus __ice_status, ::Ice::Long __ice_tries, ::Ice::Float __ice_priority, ::Ice::Float __ice_costs, ::Ice::Double __ice_informationGain, ::Ice::Int __ice_rank, const ::std::string& __ice_goal) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#else
    ::motivation::slice::Motive(__ice_created, __ice_updated, __ice_referenceEntry, __ice_correspondingUnion, __ice_thisEntry, __ice_status, __ice_tries, __ice_priority, __ice_costs, __ice_informationGain, __ice_rank, __ice_goal)
#endif

{
}

::Ice::ObjectPtr
motivation::slice::CategorizeRoomMotive::ice_clone() const
{
    ::motivation::slice::CategorizeRoomMotivePtr __p = new ::motivation::slice::CategorizeRoomMotive(*this);
    return __p;
}

static const ::std::string __motivation__slice__CategorizeRoomMotive_ids[3] =
{
    "::Ice::Object",
    "::motivation::slice::CategorizeRoomMotive",
    "::motivation::slice::Motive"
};

bool
motivation::slice::CategorizeRoomMotive::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__CategorizeRoomMotive_ids, __motivation__slice__CategorizeRoomMotive_ids + 3, _s);
}

::std::vector< ::std::string>
motivation::slice::CategorizeRoomMotive::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__CategorizeRoomMotive_ids[0], &__motivation__slice__CategorizeRoomMotive_ids[3]);
}

const ::std::string&
motivation::slice::CategorizeRoomMotive::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__CategorizeRoomMotive_ids[1];
}

const ::std::string&
motivation::slice::CategorizeRoomMotive::ice_staticId()
{
    return __motivation__slice__CategorizeRoomMotive_ids[1];
}

void
motivation::slice::CategorizeRoomMotive::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__os);
#else
    ::motivation::slice::Motive::__write(__os);
#endif
}

void
motivation::slice::CategorizeRoomMotive::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__is, true);
#else
    ::motivation::slice::Motive::__read(__is, true);
#endif
}

void
motivation::slice::CategorizeRoomMotive::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__write(__outS);
#else
    ::motivation::slice::Motive::__write(__outS);
#endif
}

void
motivation::slice::CategorizeRoomMotive::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Motive::__read(__inS, true);
#else
    ::motivation::slice::Motive::__read(__inS, true);
#endif
}

class __F__motivation__slice__CategorizeRoomMotive : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::CategorizeRoomMotive::ice_staticId());
        return new ::motivation::slice::CategorizeRoomMotive;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__CategorizeRoomMotive_Ptr = new __F__motivation__slice__CategorizeRoomMotive;

const ::Ice::ObjectFactoryPtr&
motivation::slice::CategorizeRoomMotive::ice_factory()
{
    return __F__motivation__slice__CategorizeRoomMotive_Ptr;
}

class __F__motivation__slice__CategorizeRoomMotive__Init
{
public:

    __F__motivation__slice__CategorizeRoomMotive__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::CategorizeRoomMotive::ice_staticId(), ::motivation::slice::CategorizeRoomMotive::ice_factory());
    }

    ~__F__motivation__slice__CategorizeRoomMotive__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::CategorizeRoomMotive::ice_staticId());
    }
};

static __F__motivation__slice__CategorizeRoomMotive__Init __F__motivation__slice__CategorizeRoomMotive__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__CategorizeRoomMotive__initializer() {} }
#endif

void 
motivation::slice::__patch__CategorizeRoomMotivePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::CategorizeRoomMotivePtr* p = static_cast< ::motivation::slice::CategorizeRoomMotivePtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::CategorizeRoomMotivePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::CategorizeRoomMotive::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::CategorizeRoomMotive& l, const ::motivation::slice::CategorizeRoomMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::CategorizeRoomMotive& l, const ::motivation::slice::CategorizeRoomMotive& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

motivation::slice::PlanProxy::PlanProxy(const ::cast::cdl::WorkingMemoryAddress& __ice_planAddress) :
    planAddress(__ice_planAddress)
{
}

::Ice::ObjectPtr
motivation::slice::PlanProxy::ice_clone() const
{
    ::motivation::slice::PlanProxyPtr __p = new ::motivation::slice::PlanProxy(*this);
    return __p;
}

static const ::std::string __motivation__slice__PlanProxy_ids[2] =
{
    "::Ice::Object",
    "::motivation::slice::PlanProxy"
};

bool
motivation::slice::PlanProxy::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__motivation__slice__PlanProxy_ids, __motivation__slice__PlanProxy_ids + 2, _s);
}

::std::vector< ::std::string>
motivation::slice::PlanProxy::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__motivation__slice__PlanProxy_ids[0], &__motivation__slice__PlanProxy_ids[2]);
}

const ::std::string&
motivation::slice::PlanProxy::ice_id(const ::Ice::Current&) const
{
    return __motivation__slice__PlanProxy_ids[1];
}

const ::std::string&
motivation::slice::PlanProxy::ice_staticId()
{
    return __motivation__slice__PlanProxy_ids[1];
}

void
motivation::slice::PlanProxy::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    planAddress.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
motivation::slice::PlanProxy::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    planAddress.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
motivation::slice::PlanProxy::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    ::cast::cdl::ice_writeWorkingMemoryAddress(__outS, planAddress);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
motivation::slice::PlanProxy::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    ::cast::cdl::ice_readWorkingMemoryAddress(__inS, planAddress);
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__motivation__slice__PlanProxy : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::motivation::slice::PlanProxy::ice_staticId());
        return new ::motivation::slice::PlanProxy;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__motivation__slice__PlanProxy_Ptr = new __F__motivation__slice__PlanProxy;

const ::Ice::ObjectFactoryPtr&
motivation::slice::PlanProxy::ice_factory()
{
    return __F__motivation__slice__PlanProxy_Ptr;
}

class __F__motivation__slice__PlanProxy__Init
{
public:

    __F__motivation__slice__PlanProxy__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::motivation::slice::PlanProxy::ice_staticId(), ::motivation::slice::PlanProxy::ice_factory());
    }

    ~__F__motivation__slice__PlanProxy__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::motivation::slice::PlanProxy::ice_staticId());
    }
};

static __F__motivation__slice__PlanProxy__Init __F__motivation__slice__PlanProxy__i;

#ifdef __APPLE__
extern "C" { void __F__motivation__slice__PlanProxy__initializer() {} }
#endif

void 
motivation::slice::__patch__PlanProxyPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::motivation::slice::PlanProxyPtr* p = static_cast< ::motivation::slice::PlanProxyPtr*>(__addr);
    assert(p);
    *p = ::motivation::slice::PlanProxyPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::motivation::slice::PlanProxy::ice_staticId(), v->ice_id());
    }
}

bool
motivation::slice::operator==(const ::motivation::slice::PlanProxy& l, const ::motivation::slice::PlanProxy& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
motivation::slice::operator<(const ::motivation::slice::PlanProxy& l, const ::motivation::slice::PlanProxy& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
