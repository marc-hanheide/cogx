// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `Execution.ice'

#include <Execution.hpp>
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

::Ice::Object* IceInternal::upCast(::execution::slice::Action* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::execution::slice::Action* p) { return p; }

::Ice::Object* IceInternal::upCast(::execution::slice::actions::GoToPlace* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::execution::slice::actions::GoToPlace* p) { return p; }

::Ice::Object* IceInternal::upCast(::execution::slice::actions::ActiveVisualSearch* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::execution::slice::actions::ActiveVisualSearch* p) { return p; }

::Ice::Object* IceInternal::upCast(::execution::slice::actions::ExplorePlace* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::execution::slice::actions::ExplorePlace* p) { return p; }

::Ice::Object* IceInternal::upCast(::execution::slice::actions::PrintMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::execution::slice::actions::PrintMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::execution::slice::actions::LogMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::execution::slice::actions::LogMessage* p) { return p; }

void
execution::slice::__read(::IceInternal::BasicStream* __is, ::execution::slice::ActionPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::Action;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::ice_writeActionPrx(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::ActionPrx& v)
{
    __outS->writeProxy(v);
}

void
execution::slice::ice_readActionPrx(const ::Ice::InputStreamPtr& __inS, ::execution::slice::ActionPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::Action;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::ice_writeAction(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::ActionPtr& v)
{
    __outS->writeObject(v);
}

void
execution::slice::ice_readAction(const ::Ice::InputStreamPtr& __inS, ::execution::slice::ActionPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::execution::slice::__patch__ActionPtr, &__v);
    __inS->readObject(__cb);
}

void
execution::slice::actions::__read(::IceInternal::BasicStream* __is, ::execution::slice::actions::GoToPlacePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::GoToPlace;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeGoToPlacePrx(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::GoToPlacePrx& v)
{
    __outS->writeProxy(v);
}

void
execution::slice::actions::ice_readGoToPlacePrx(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::GoToPlacePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::GoToPlace;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeGoToPlace(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::GoToPlacePtr& v)
{
    __outS->writeObject(v);
}

void
execution::slice::actions::ice_readGoToPlace(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::GoToPlacePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::execution::slice::actions::__patch__GoToPlacePtr, &__v);
    __inS->readObject(__cb);
}

void
execution::slice::actions::__read(::IceInternal::BasicStream* __is, ::execution::slice::actions::ActiveVisualSearchPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::ActiveVisualSearch;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeActiveVisualSearchPrx(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::ActiveVisualSearchPrx& v)
{
    __outS->writeProxy(v);
}

void
execution::slice::actions::ice_readActiveVisualSearchPrx(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::ActiveVisualSearchPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::ActiveVisualSearch;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeActiveVisualSearch(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::ActiveVisualSearchPtr& v)
{
    __outS->writeObject(v);
}

void
execution::slice::actions::ice_readActiveVisualSearch(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::ActiveVisualSearchPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::execution::slice::actions::__patch__ActiveVisualSearchPtr, &__v);
    __inS->readObject(__cb);
}

void
execution::slice::actions::__read(::IceInternal::BasicStream* __is, ::execution::slice::actions::ExplorePlacePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::ExplorePlace;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeExplorePlacePrx(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::ExplorePlacePrx& v)
{
    __outS->writeProxy(v);
}

void
execution::slice::actions::ice_readExplorePlacePrx(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::ExplorePlacePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::ExplorePlace;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeExplorePlace(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::ExplorePlacePtr& v)
{
    __outS->writeObject(v);
}

void
execution::slice::actions::ice_readExplorePlace(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::ExplorePlacePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::execution::slice::actions::__patch__ExplorePlacePtr, &__v);
    __inS->readObject(__cb);
}

void
execution::slice::actions::__read(::IceInternal::BasicStream* __is, ::execution::slice::actions::PrintMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::PrintMessage;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writePrintMessagePrx(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::PrintMessagePrx& v)
{
    __outS->writeProxy(v);
}

void
execution::slice::actions::ice_readPrintMessagePrx(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::PrintMessagePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::PrintMessage;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writePrintMessage(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::PrintMessagePtr& v)
{
    __outS->writeObject(v);
}

void
execution::slice::actions::ice_readPrintMessage(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::PrintMessagePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::execution::slice::actions::__patch__PrintMessagePtr, &__v);
    __inS->readObject(__cb);
}

void
execution::slice::actions::__read(::IceInternal::BasicStream* __is, ::execution::slice::actions::LogMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::LogMessage;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeLogMessagePrx(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::LogMessagePrx& v)
{
    __outS->writeProxy(v);
}

void
execution::slice::actions::ice_readLogMessagePrx(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::LogMessagePrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::execution::slice::actions::LogMessage;
        v->__copyFrom(proxy);
    }
}

void
execution::slice::actions::ice_writeLogMessage(const ::Ice::OutputStreamPtr& __outS, const ::execution::slice::actions::LogMessagePtr& v)
{
    __outS->writeObject(v);
}

void
execution::slice::actions::ice_readLogMessage(const ::Ice::InputStreamPtr& __inS, ::execution::slice::actions::LogMessagePtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::execution::slice::actions::__patch__LogMessagePtr, &__v);
    __inS->readObject(__cb);
}

void
execution::slice::__write(::IceInternal::BasicStream* __os, ::execution::slice::TriBool v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 3);
}

void
execution::slice::__read(::IceInternal::BasicStream* __is, ::execution::slice::TriBool& v)
{
    ::Ice::Byte val;
    __is->read(val, 3);
    v = static_cast< ::execution::slice::TriBool>(val);
}

void
execution::slice::ice_writeTriBool(const ::Ice::OutputStreamPtr& __outS, ::execution::slice::TriBool v)
{
    if(static_cast<int>(v) >= 3)
    {
        throw ::Ice::MarshalException(__FILE__, __LINE__, "enumerator out of range");
    }
    __outS->writeByte(static_cast< ::Ice::Byte>(v));
}

void
execution::slice::ice_readTriBool(const ::Ice::InputStreamPtr& __inS, ::execution::slice::TriBool& v)
{
    ::Ice::Byte val = __inS->readByte();
    if(val > 3)
    {
        throw ::Ice::MarshalException(__FILE__, __LINE__, "enumerator out of range");
    }
    v = static_cast< ::execution::slice::TriBool>(val);
}

void
execution::slice::__write(::IceInternal::BasicStream* __os, ::execution::slice::ActionStatus v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 4);
}

void
execution::slice::__read(::IceInternal::BasicStream* __is, ::execution::slice::ActionStatus& v)
{
    ::Ice::Byte val;
    __is->read(val, 4);
    v = static_cast< ::execution::slice::ActionStatus>(val);
}

void
execution::slice::ice_writeActionStatus(const ::Ice::OutputStreamPtr& __outS, ::execution::slice::ActionStatus v)
{
    if(static_cast<int>(v) >= 4)
    {
        throw ::Ice::MarshalException(__FILE__, __LINE__, "enumerator out of range");
    }
    __outS->writeByte(static_cast< ::Ice::Byte>(v));
}

void
execution::slice::ice_readActionStatus(const ::Ice::InputStreamPtr& __inS, ::execution::slice::ActionStatus& v)
{
    ::Ice::Byte val = __inS->readByte();
    if(val > 4)
    {
        throw ::Ice::MarshalException(__FILE__, __LINE__, "enumerator out of range");
    }
    v = static_cast< ::execution::slice::ActionStatus>(val);
}

execution::slice::ActionExecutionException::ActionExecutionException(const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException(__ice_message)
#else
    ::cast::CASTException(__ice_message)
#endif
{
}

execution::slice::ActionExecutionException::~ActionExecutionException() throw()
{
}

static const char* __execution__slice__ActionExecutionException_name = "execution::slice::ActionExecutionException";

::std::string
execution::slice::ActionExecutionException::ice_name() const
{
    return __execution__slice__ActionExecutionException_name;
}

::Ice::Exception*
execution::slice::ActionExecutionException::ice_clone() const
{
    return new ActionExecutionException(*this);
}

void
execution::slice::ActionExecutionException::ice_throw() const
{
    throw *this;
}

void
execution::slice::ActionExecutionException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::execution::slice::ActionExecutionException"), false);
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__write(__os);
#else
    ::cast::CASTException::__write(__os);
#endif
}

void
execution::slice::ActionExecutionException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__read(__is, true);
#else
    ::cast::CASTException::__read(__is, true);
#endif
}

void
execution::slice::ActionExecutionException::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeString(::std::string("::execution::slice::ActionExecutionException"));
    __outS->startSlice();
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__write(__outS);
#else
    ::cast::CASTException::__write(__outS);
#endif
}

void
execution::slice::ActionExecutionException::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readString();
    }
    __inS->startSlice();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    CASTException::__read(__inS, true);
#else
    ::cast::CASTException::__read(__inS, true);
#endif
}

struct __F__execution__slice__ActionExecutionException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::execution::slice::ActionExecutionException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__execution__slice__ActionExecutionException__Ptr = new __F__execution__slice__ActionExecutionException;

const ::IceInternal::UserExceptionFactoryPtr&
execution::slice::ActionExecutionException::ice_factory()
{
    return __F__execution__slice__ActionExecutionException__Ptr;
}

class __F__execution__slice__ActionExecutionException__Init
{
public:

    __F__execution__slice__ActionExecutionException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::execution::slice::ActionExecutionException", ::execution::slice::ActionExecutionException::ice_factory());
    }

    ~__F__execution__slice__ActionExecutionException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::execution::slice::ActionExecutionException");
    }
};

static __F__execution__slice__ActionExecutionException__Init __F__execution__slice__ActionExecutionException__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__ActionExecutionException__initializer() {} }
#endif

const ::std::string&
IceProxy::execution::slice::Action::ice_staticId()
{
    return ::execution::slice::Action::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::execution::slice::Action::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::execution::slice::Action);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::execution::slice::Action::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::execution::slice::Action);
}

::IceProxy::Ice::Object*
IceProxy::execution::slice::Action::__newInstance() const
{
    return new Action;
}

const ::std::string&
IceProxy::execution::slice::actions::GoToPlace::ice_staticId()
{
    return ::execution::slice::actions::GoToPlace::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::execution::slice::actions::GoToPlace::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::execution::slice::actions::GoToPlace);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::execution::slice::actions::GoToPlace::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::execution::slice::actions::GoToPlace);
}

::IceProxy::Ice::Object*
IceProxy::execution::slice::actions::GoToPlace::__newInstance() const
{
    return new GoToPlace;
}

const ::std::string&
IceProxy::execution::slice::actions::ActiveVisualSearch::ice_staticId()
{
    return ::execution::slice::actions::ActiveVisualSearch::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::execution::slice::actions::ActiveVisualSearch::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::execution::slice::actions::ActiveVisualSearch);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::execution::slice::actions::ActiveVisualSearch::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::execution::slice::actions::ActiveVisualSearch);
}

::IceProxy::Ice::Object*
IceProxy::execution::slice::actions::ActiveVisualSearch::__newInstance() const
{
    return new ActiveVisualSearch;
}

const ::std::string&
IceProxy::execution::slice::actions::ExplorePlace::ice_staticId()
{
    return ::execution::slice::actions::ExplorePlace::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::execution::slice::actions::ExplorePlace::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::execution::slice::actions::ExplorePlace);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::execution::slice::actions::ExplorePlace::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::execution::slice::actions::ExplorePlace);
}

::IceProxy::Ice::Object*
IceProxy::execution::slice::actions::ExplorePlace::__newInstance() const
{
    return new ExplorePlace;
}

const ::std::string&
IceProxy::execution::slice::actions::PrintMessage::ice_staticId()
{
    return ::execution::slice::actions::PrintMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::execution::slice::actions::PrintMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::execution::slice::actions::PrintMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::execution::slice::actions::PrintMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::execution::slice::actions::PrintMessage);
}

::IceProxy::Ice::Object*
IceProxy::execution::slice::actions::PrintMessage::__newInstance() const
{
    return new PrintMessage;
}

const ::std::string&
IceProxy::execution::slice::actions::LogMessage::ice_staticId()
{
    return ::execution::slice::actions::LogMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::execution::slice::actions::LogMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::execution::slice::actions::LogMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::execution::slice::actions::LogMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::execution::slice::actions::LogMessage);
}

::IceProxy::Ice::Object*
IceProxy::execution::slice::actions::LogMessage::__newInstance() const
{
    return new LogMessage;
}

execution::slice::Action::Action(::execution::slice::ActionStatus __ice_status, ::execution::slice::TriBool __ice_success) :
    status(__ice_status),
    success(__ice_success)
{
}

::Ice::ObjectPtr
execution::slice::Action::ice_clone() const
{
    ::execution::slice::ActionPtr __p = new ::execution::slice::Action(*this);
    return __p;
}

static const ::std::string __execution__slice__Action_ids[2] =
{
    "::Ice::Object",
    "::execution::slice::Action"
};

bool
execution::slice::Action::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__execution__slice__Action_ids, __execution__slice__Action_ids + 2, _s);
}

::std::vector< ::std::string>
execution::slice::Action::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__execution__slice__Action_ids[0], &__execution__slice__Action_ids[2]);
}

const ::std::string&
execution::slice::Action::ice_id(const ::Ice::Current&) const
{
    return __execution__slice__Action_ids[1];
}

const ::std::string&
execution::slice::Action::ice_staticId()
{
    return __execution__slice__Action_ids[1];
}

void
execution::slice::Action::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    ::execution::slice::__write(__os, status);
    ::execution::slice::__write(__os, success);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
execution::slice::Action::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    ::execution::slice::__read(__is, status);
    ::execution::slice::__read(__is, success);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
execution::slice::Action::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    ::execution::slice::ice_writeActionStatus(__outS, status);
    ::execution::slice::ice_writeTriBool(__outS, success);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
execution::slice::Action::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    ::execution::slice::ice_readActionStatus(__inS, status);
    ::execution::slice::ice_readTriBool(__inS, success);
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__execution__slice__Action : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::execution::slice::Action::ice_staticId());
        return new ::execution::slice::Action;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__execution__slice__Action_Ptr = new __F__execution__slice__Action;

const ::Ice::ObjectFactoryPtr&
execution::slice::Action::ice_factory()
{
    return __F__execution__slice__Action_Ptr;
}

class __F__execution__slice__Action__Init
{
public:

    __F__execution__slice__Action__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::execution::slice::Action::ice_staticId(), ::execution::slice::Action::ice_factory());
    }

    ~__F__execution__slice__Action__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::execution::slice::Action::ice_staticId());
    }
};

static __F__execution__slice__Action__Init __F__execution__slice__Action__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__Action__initializer() {} }
#endif

void 
execution::slice::__patch__ActionPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::execution::slice::ActionPtr* p = static_cast< ::execution::slice::ActionPtr*>(__addr);
    assert(p);
    *p = ::execution::slice::ActionPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::execution::slice::Action::ice_staticId(), v->ice_id());
    }
}

bool
execution::slice::operator==(const ::execution::slice::Action& l, const ::execution::slice::Action& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
execution::slice::operator<(const ::execution::slice::Action& l, const ::execution::slice::Action& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

execution::slice::actions::GoToPlace::GoToPlace(::execution::slice::ActionStatus __ice_status, ::execution::slice::TriBool __ice_success, ::Ice::Long __ice_placeID) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action(__ice_status, __ice_success)
#else
    ::execution::slice::Action(__ice_status, __ice_success)
#endif
,
    placeID(__ice_placeID)
{
}

::Ice::ObjectPtr
execution::slice::actions::GoToPlace::ice_clone() const
{
    ::execution::slice::actions::GoToPlacePtr __p = new ::execution::slice::actions::GoToPlace(*this);
    return __p;
}

static const ::std::string __execution__slice__actions__GoToPlace_ids[3] =
{
    "::Ice::Object",
    "::execution::slice::Action",
    "::execution::slice::actions::GoToPlace"
};

bool
execution::slice::actions::GoToPlace::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__execution__slice__actions__GoToPlace_ids, __execution__slice__actions__GoToPlace_ids + 3, _s);
}

::std::vector< ::std::string>
execution::slice::actions::GoToPlace::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__execution__slice__actions__GoToPlace_ids[0], &__execution__slice__actions__GoToPlace_ids[3]);
}

const ::std::string&
execution::slice::actions::GoToPlace::ice_id(const ::Ice::Current&) const
{
    return __execution__slice__actions__GoToPlace_ids[2];
}

const ::std::string&
execution::slice::actions::GoToPlace::ice_staticId()
{
    return __execution__slice__actions__GoToPlace_ids[2];
}

void
execution::slice::actions::GoToPlace::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(placeID);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__os);
#else
    ::execution::slice::Action::__write(__os);
#endif
}

void
execution::slice::actions::GoToPlace::__read(::IceInternal::BasicStream* __is, bool __rid)
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
    Action::__read(__is, true);
#else
    ::execution::slice::Action::__read(__is, true);
#endif
}

void
execution::slice::actions::GoToPlace::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeLong(placeID);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__outS);
#else
    ::execution::slice::Action::__write(__outS);
#endif
}

void
execution::slice::actions::GoToPlace::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    placeID = __inS->readLong();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__inS, true);
#else
    ::execution::slice::Action::__read(__inS, true);
#endif
}

class __F__execution__slice__actions__GoToPlace : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::execution::slice::actions::GoToPlace::ice_staticId());
        return new ::execution::slice::actions::GoToPlace;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__execution__slice__actions__GoToPlace_Ptr = new __F__execution__slice__actions__GoToPlace;

const ::Ice::ObjectFactoryPtr&
execution::slice::actions::GoToPlace::ice_factory()
{
    return __F__execution__slice__actions__GoToPlace_Ptr;
}

class __F__execution__slice__actions__GoToPlace__Init
{
public:

    __F__execution__slice__actions__GoToPlace__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::execution::slice::actions::GoToPlace::ice_staticId(), ::execution::slice::actions::GoToPlace::ice_factory());
    }

    ~__F__execution__slice__actions__GoToPlace__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::execution::slice::actions::GoToPlace::ice_staticId());
    }
};

static __F__execution__slice__actions__GoToPlace__Init __F__execution__slice__actions__GoToPlace__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__actions__GoToPlace__initializer() {} }
#endif

void 
execution::slice::actions::__patch__GoToPlacePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::execution::slice::actions::GoToPlacePtr* p = static_cast< ::execution::slice::actions::GoToPlacePtr*>(__addr);
    assert(p);
    *p = ::execution::slice::actions::GoToPlacePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::execution::slice::actions::GoToPlace::ice_staticId(), v->ice_id());
    }
}

bool
execution::slice::actions::operator==(const ::execution::slice::actions::GoToPlace& l, const ::execution::slice::actions::GoToPlace& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
execution::slice::actions::operator<(const ::execution::slice::actions::GoToPlace& l, const ::execution::slice::actions::GoToPlace& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

execution::slice::actions::ActiveVisualSearch::ActiveVisualSearch(::execution::slice::ActionStatus __ice_status, ::execution::slice::TriBool __ice_success, const ::execution::slice::actions::LongSeq& __ice_placeIDs) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action(__ice_status, __ice_success)
#else
    ::execution::slice::Action(__ice_status, __ice_success)
#endif
,
    placeIDs(__ice_placeIDs)
{
}

::Ice::ObjectPtr
execution::slice::actions::ActiveVisualSearch::ice_clone() const
{
    ::execution::slice::actions::ActiveVisualSearchPtr __p = new ::execution::slice::actions::ActiveVisualSearch(*this);
    return __p;
}

static const ::std::string __execution__slice__actions__ActiveVisualSearch_ids[3] =
{
    "::Ice::Object",
    "::execution::slice::Action",
    "::execution::slice::actions::ActiveVisualSearch"
};

bool
execution::slice::actions::ActiveVisualSearch::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__execution__slice__actions__ActiveVisualSearch_ids, __execution__slice__actions__ActiveVisualSearch_ids + 3, _s);
}

::std::vector< ::std::string>
execution::slice::actions::ActiveVisualSearch::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__execution__slice__actions__ActiveVisualSearch_ids[0], &__execution__slice__actions__ActiveVisualSearch_ids[3]);
}

const ::std::string&
execution::slice::actions::ActiveVisualSearch::ice_id(const ::Ice::Current&) const
{
    return __execution__slice__actions__ActiveVisualSearch_ids[2];
}

const ::std::string&
execution::slice::actions::ActiveVisualSearch::ice_staticId()
{
    return __execution__slice__actions__ActiveVisualSearch_ids[2];
}

void
execution::slice::actions::ActiveVisualSearch::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    if(placeIDs.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&placeIDs[0], &placeIDs[0] + placeIDs.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__os);
#else
    ::execution::slice::Action::__write(__os);
#endif
}

void
execution::slice::actions::ActiveVisualSearch::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(placeIDs);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__is, true);
#else
    ::execution::slice::Action::__read(__is, true);
#endif
}

void
execution::slice::actions::ActiveVisualSearch::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeLongSeq(placeIDs);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__outS);
#else
    ::execution::slice::Action::__write(__outS);
#endif
}

void
execution::slice::actions::ActiveVisualSearch::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    placeIDs = __inS->readLongSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__inS, true);
#else
    ::execution::slice::Action::__read(__inS, true);
#endif
}

class __F__execution__slice__actions__ActiveVisualSearch : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::execution::slice::actions::ActiveVisualSearch::ice_staticId());
        return new ::execution::slice::actions::ActiveVisualSearch;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__execution__slice__actions__ActiveVisualSearch_Ptr = new __F__execution__slice__actions__ActiveVisualSearch;

const ::Ice::ObjectFactoryPtr&
execution::slice::actions::ActiveVisualSearch::ice_factory()
{
    return __F__execution__slice__actions__ActiveVisualSearch_Ptr;
}

class __F__execution__slice__actions__ActiveVisualSearch__Init
{
public:

    __F__execution__slice__actions__ActiveVisualSearch__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::execution::slice::actions::ActiveVisualSearch::ice_staticId(), ::execution::slice::actions::ActiveVisualSearch::ice_factory());
    }

    ~__F__execution__slice__actions__ActiveVisualSearch__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::execution::slice::actions::ActiveVisualSearch::ice_staticId());
    }
};

static __F__execution__slice__actions__ActiveVisualSearch__Init __F__execution__slice__actions__ActiveVisualSearch__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__actions__ActiveVisualSearch__initializer() {} }
#endif

void 
execution::slice::actions::__patch__ActiveVisualSearchPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::execution::slice::actions::ActiveVisualSearchPtr* p = static_cast< ::execution::slice::actions::ActiveVisualSearchPtr*>(__addr);
    assert(p);
    *p = ::execution::slice::actions::ActiveVisualSearchPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::execution::slice::actions::ActiveVisualSearch::ice_staticId(), v->ice_id());
    }
}

bool
execution::slice::actions::operator==(const ::execution::slice::actions::ActiveVisualSearch& l, const ::execution::slice::actions::ActiveVisualSearch& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
execution::slice::actions::operator<(const ::execution::slice::actions::ActiveVisualSearch& l, const ::execution::slice::actions::ActiveVisualSearch& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

execution::slice::actions::ExplorePlace::ExplorePlace(::execution::slice::ActionStatus __ice_status, ::execution::slice::TriBool __ice_success) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action(__ice_status, __ice_success)
#else
    ::execution::slice::Action(__ice_status, __ice_success)
#endif

{
}

::Ice::ObjectPtr
execution::slice::actions::ExplorePlace::ice_clone() const
{
    ::execution::slice::actions::ExplorePlacePtr __p = new ::execution::slice::actions::ExplorePlace(*this);
    return __p;
}

static const ::std::string __execution__slice__actions__ExplorePlace_ids[3] =
{
    "::Ice::Object",
    "::execution::slice::Action",
    "::execution::slice::actions::ExplorePlace"
};

bool
execution::slice::actions::ExplorePlace::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__execution__slice__actions__ExplorePlace_ids, __execution__slice__actions__ExplorePlace_ids + 3, _s);
}

::std::vector< ::std::string>
execution::slice::actions::ExplorePlace::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__execution__slice__actions__ExplorePlace_ids[0], &__execution__slice__actions__ExplorePlace_ids[3]);
}

const ::std::string&
execution::slice::actions::ExplorePlace::ice_id(const ::Ice::Current&) const
{
    return __execution__slice__actions__ExplorePlace_ids[2];
}

const ::std::string&
execution::slice::actions::ExplorePlace::ice_staticId()
{
    return __execution__slice__actions__ExplorePlace_ids[2];
}

void
execution::slice::actions::ExplorePlace::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__os);
#else
    ::execution::slice::Action::__write(__os);
#endif
}

void
execution::slice::actions::ExplorePlace::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__is, true);
#else
    ::execution::slice::Action::__read(__is, true);
#endif
}

void
execution::slice::actions::ExplorePlace::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__outS);
#else
    ::execution::slice::Action::__write(__outS);
#endif
}

void
execution::slice::actions::ExplorePlace::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__inS, true);
#else
    ::execution::slice::Action::__read(__inS, true);
#endif
}

class __F__execution__slice__actions__ExplorePlace : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::execution::slice::actions::ExplorePlace::ice_staticId());
        return new ::execution::slice::actions::ExplorePlace;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__execution__slice__actions__ExplorePlace_Ptr = new __F__execution__slice__actions__ExplorePlace;

const ::Ice::ObjectFactoryPtr&
execution::slice::actions::ExplorePlace::ice_factory()
{
    return __F__execution__slice__actions__ExplorePlace_Ptr;
}

class __F__execution__slice__actions__ExplorePlace__Init
{
public:

    __F__execution__slice__actions__ExplorePlace__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::execution::slice::actions::ExplorePlace::ice_staticId(), ::execution::slice::actions::ExplorePlace::ice_factory());
    }

    ~__F__execution__slice__actions__ExplorePlace__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::execution::slice::actions::ExplorePlace::ice_staticId());
    }
};

static __F__execution__slice__actions__ExplorePlace__Init __F__execution__slice__actions__ExplorePlace__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__actions__ExplorePlace__initializer() {} }
#endif

void 
execution::slice::actions::__patch__ExplorePlacePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::execution::slice::actions::ExplorePlacePtr* p = static_cast< ::execution::slice::actions::ExplorePlacePtr*>(__addr);
    assert(p);
    *p = ::execution::slice::actions::ExplorePlacePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::execution::slice::actions::ExplorePlace::ice_staticId(), v->ice_id());
    }
}

bool
execution::slice::actions::operator==(const ::execution::slice::actions::ExplorePlace& l, const ::execution::slice::actions::ExplorePlace& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
execution::slice::actions::operator<(const ::execution::slice::actions::ExplorePlace& l, const ::execution::slice::actions::ExplorePlace& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

execution::slice::actions::PrintMessage::PrintMessage(::execution::slice::ActionStatus __ice_status, ::execution::slice::TriBool __ice_success, const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action(__ice_status, __ice_success)
#else
    ::execution::slice::Action(__ice_status, __ice_success)
#endif
,
    message(__ice_message)
{
}

::Ice::ObjectPtr
execution::slice::actions::PrintMessage::ice_clone() const
{
    ::execution::slice::actions::PrintMessagePtr __p = new ::execution::slice::actions::PrintMessage(*this);
    return __p;
}

static const ::std::string __execution__slice__actions__PrintMessage_ids[3] =
{
    "::Ice::Object",
    "::execution::slice::Action",
    "::execution::slice::actions::PrintMessage"
};

bool
execution::slice::actions::PrintMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__execution__slice__actions__PrintMessage_ids, __execution__slice__actions__PrintMessage_ids + 3, _s);
}

::std::vector< ::std::string>
execution::slice::actions::PrintMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__execution__slice__actions__PrintMessage_ids[0], &__execution__slice__actions__PrintMessage_ids[3]);
}

const ::std::string&
execution::slice::actions::PrintMessage::ice_id(const ::Ice::Current&) const
{
    return __execution__slice__actions__PrintMessage_ids[2];
}

const ::std::string&
execution::slice::actions::PrintMessage::ice_staticId()
{
    return __execution__slice__actions__PrintMessage_ids[2];
}

void
execution::slice::actions::PrintMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(message);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__os);
#else
    ::execution::slice::Action::__write(__os);
#endif
}

void
execution::slice::actions::PrintMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(message);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__is, true);
#else
    ::execution::slice::Action::__read(__is, true);
#endif
}

void
execution::slice::actions::PrintMessage::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(message);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__outS);
#else
    ::execution::slice::Action::__write(__outS);
#endif
}

void
execution::slice::actions::PrintMessage::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    message = __inS->readString();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__inS, true);
#else
    ::execution::slice::Action::__read(__inS, true);
#endif
}

class __F__execution__slice__actions__PrintMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::execution::slice::actions::PrintMessage::ice_staticId());
        return new ::execution::slice::actions::PrintMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__execution__slice__actions__PrintMessage_Ptr = new __F__execution__slice__actions__PrintMessage;

const ::Ice::ObjectFactoryPtr&
execution::slice::actions::PrintMessage::ice_factory()
{
    return __F__execution__slice__actions__PrintMessage_Ptr;
}

class __F__execution__slice__actions__PrintMessage__Init
{
public:

    __F__execution__slice__actions__PrintMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::execution::slice::actions::PrintMessage::ice_staticId(), ::execution::slice::actions::PrintMessage::ice_factory());
    }

    ~__F__execution__slice__actions__PrintMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::execution::slice::actions::PrintMessage::ice_staticId());
    }
};

static __F__execution__slice__actions__PrintMessage__Init __F__execution__slice__actions__PrintMessage__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__actions__PrintMessage__initializer() {} }
#endif

void 
execution::slice::actions::__patch__PrintMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::execution::slice::actions::PrintMessagePtr* p = static_cast< ::execution::slice::actions::PrintMessagePtr*>(__addr);
    assert(p);
    *p = ::execution::slice::actions::PrintMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::execution::slice::actions::PrintMessage::ice_staticId(), v->ice_id());
    }
}

bool
execution::slice::actions::operator==(const ::execution::slice::actions::PrintMessage& l, const ::execution::slice::actions::PrintMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
execution::slice::actions::operator<(const ::execution::slice::actions::PrintMessage& l, const ::execution::slice::actions::PrintMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

execution::slice::actions::LogMessage::LogMessage(::execution::slice::ActionStatus __ice_status, ::execution::slice::TriBool __ice_success, const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action(__ice_status, __ice_success)
#else
    ::execution::slice::Action(__ice_status, __ice_success)
#endif
,
    message(__ice_message)
{
}

::Ice::ObjectPtr
execution::slice::actions::LogMessage::ice_clone() const
{
    ::execution::slice::actions::LogMessagePtr __p = new ::execution::slice::actions::LogMessage(*this);
    return __p;
}

static const ::std::string __execution__slice__actions__LogMessage_ids[3] =
{
    "::Ice::Object",
    "::execution::slice::Action",
    "::execution::slice::actions::LogMessage"
};

bool
execution::slice::actions::LogMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__execution__slice__actions__LogMessage_ids, __execution__slice__actions__LogMessage_ids + 3, _s);
}

::std::vector< ::std::string>
execution::slice::actions::LogMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__execution__slice__actions__LogMessage_ids[0], &__execution__slice__actions__LogMessage_ids[3]);
}

const ::std::string&
execution::slice::actions::LogMessage::ice_id(const ::Ice::Current&) const
{
    return __execution__slice__actions__LogMessage_ids[2];
}

const ::std::string&
execution::slice::actions::LogMessage::ice_staticId()
{
    return __execution__slice__actions__LogMessage_ids[2];
}

void
execution::slice::actions::LogMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(message);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__os);
#else
    ::execution::slice::Action::__write(__os);
#endif
}

void
execution::slice::actions::LogMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(message);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__is, true);
#else
    ::execution::slice::Action::__read(__is, true);
#endif
}

void
execution::slice::actions::LogMessage::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(message);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__write(__outS);
#else
    ::execution::slice::Action::__write(__outS);
#endif
}

void
execution::slice::actions::LogMessage::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    message = __inS->readString();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Action::__read(__inS, true);
#else
    ::execution::slice::Action::__read(__inS, true);
#endif
}

class __F__execution__slice__actions__LogMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::execution::slice::actions::LogMessage::ice_staticId());
        return new ::execution::slice::actions::LogMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__execution__slice__actions__LogMessage_Ptr = new __F__execution__slice__actions__LogMessage;

const ::Ice::ObjectFactoryPtr&
execution::slice::actions::LogMessage::ice_factory()
{
    return __F__execution__slice__actions__LogMessage_Ptr;
}

class __F__execution__slice__actions__LogMessage__Init
{
public:

    __F__execution__slice__actions__LogMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::execution::slice::actions::LogMessage::ice_staticId(), ::execution::slice::actions::LogMessage::ice_factory());
    }

    ~__F__execution__slice__actions__LogMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::execution::slice::actions::LogMessage::ice_staticId());
    }
};

static __F__execution__slice__actions__LogMessage__Init __F__execution__slice__actions__LogMessage__i;

#ifdef __APPLE__
extern "C" { void __F__execution__slice__actions__LogMessage__initializer() {} }
#endif

void 
execution::slice::actions::__patch__LogMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::execution::slice::actions::LogMessagePtr* p = static_cast< ::execution::slice::actions::LogMessagePtr*>(__addr);
    assert(p);
    *p = ::execution::slice::actions::LogMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::execution::slice::actions::LogMessage::ice_staticId(), v->ice_id());
    }
}

bool
execution::slice::actions::operator==(const ::execution::slice::actions::LogMessage& l, const ::execution::slice::actions::LogMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
execution::slice::actions::operator<(const ::execution::slice::actions::LogMessage& l, const ::execution::slice::actions::LogMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
