// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `HelloWorldData.ice'

#include <HelloWorldData.hpp>
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

::Ice::Object* IceInternal::upCast(::helloworld::Announcement* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::helloworld::Announcement* p) { return p; }

::Ice::Object* IceInternal::upCast(::helloworld::MaryCommand* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::helloworld::MaryCommand* p) { return p; }

::Ice::Object* IceInternal::upCast(::helloworld::PlayMp3Command* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::helloworld::PlayMp3Command* p) { return p; }

void
helloworld::__read(::IceInternal::BasicStream* __is, ::helloworld::AnnouncementPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::helloworld::Announcement;
        v->__copyFrom(proxy);
    }
}

void
helloworld::ice_writeAnnouncementPrx(const ::Ice::OutputStreamPtr& __outS, const ::helloworld::AnnouncementPrx& v)
{
    __outS->writeProxy(v);
}

void
helloworld::ice_readAnnouncementPrx(const ::Ice::InputStreamPtr& __inS, ::helloworld::AnnouncementPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::helloworld::Announcement;
        v->__copyFrom(proxy);
    }
}

void
helloworld::ice_writeAnnouncement(const ::Ice::OutputStreamPtr& __outS, const ::helloworld::AnnouncementPtr& v)
{
    __outS->writeObject(v);
}

void
helloworld::ice_readAnnouncement(const ::Ice::InputStreamPtr& __inS, ::helloworld::AnnouncementPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::helloworld::__patch__AnnouncementPtr, &__v);
    __inS->readObject(__cb);
}

void
helloworld::__read(::IceInternal::BasicStream* __is, ::helloworld::MaryCommandPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::helloworld::MaryCommand;
        v->__copyFrom(proxy);
    }
}

void
helloworld::ice_writeMaryCommandPrx(const ::Ice::OutputStreamPtr& __outS, const ::helloworld::MaryCommandPrx& v)
{
    __outS->writeProxy(v);
}

void
helloworld::ice_readMaryCommandPrx(const ::Ice::InputStreamPtr& __inS, ::helloworld::MaryCommandPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::helloworld::MaryCommand;
        v->__copyFrom(proxy);
    }
}

void
helloworld::ice_writeMaryCommand(const ::Ice::OutputStreamPtr& __outS, const ::helloworld::MaryCommandPtr& v)
{
    __outS->writeObject(v);
}

void
helloworld::ice_readMaryCommand(const ::Ice::InputStreamPtr& __inS, ::helloworld::MaryCommandPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::helloworld::__patch__MaryCommandPtr, &__v);
    __inS->readObject(__cb);
}

void
helloworld::__read(::IceInternal::BasicStream* __is, ::helloworld::PlayMp3CommandPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::helloworld::PlayMp3Command;
        v->__copyFrom(proxy);
    }
}

void
helloworld::ice_writePlayMp3CommandPrx(const ::Ice::OutputStreamPtr& __outS, const ::helloworld::PlayMp3CommandPrx& v)
{
    __outS->writeProxy(v);
}

void
helloworld::ice_readPlayMp3CommandPrx(const ::Ice::InputStreamPtr& __inS, ::helloworld::PlayMp3CommandPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::helloworld::PlayMp3Command;
        v->__copyFrom(proxy);
    }
}

void
helloworld::ice_writePlayMp3Command(const ::Ice::OutputStreamPtr& __outS, const ::helloworld::PlayMp3CommandPtr& v)
{
    __outS->writeObject(v);
}

void
helloworld::ice_readPlayMp3Command(const ::Ice::InputStreamPtr& __inS, ::helloworld::PlayMp3CommandPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::helloworld::__patch__PlayMp3CommandPtr, &__v);
    __inS->readObject(__cb);
}

const ::std::string&
IceProxy::helloworld::Announcement::ice_staticId()
{
    return ::helloworld::Announcement::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::helloworld::Announcement::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::helloworld::Announcement);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::helloworld::Announcement::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::helloworld::Announcement);
}

::IceProxy::Ice::Object*
IceProxy::helloworld::Announcement::__newInstance() const
{
    return new Announcement;
}

const ::std::string&
IceProxy::helloworld::MaryCommand::ice_staticId()
{
    return ::helloworld::MaryCommand::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::helloworld::MaryCommand::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::helloworld::MaryCommand);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::helloworld::MaryCommand::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::helloworld::MaryCommand);
}

::IceProxy::Ice::Object*
IceProxy::helloworld::MaryCommand::__newInstance() const
{
    return new MaryCommand;
}

const ::std::string&
IceProxy::helloworld::PlayMp3Command::ice_staticId()
{
    return ::helloworld::PlayMp3Command::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::helloworld::PlayMp3Command::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::helloworld::PlayMp3Command);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::helloworld::PlayMp3Command::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::helloworld::PlayMp3Command);
}

::IceProxy::Ice::Object*
IceProxy::helloworld::PlayMp3Command::__newInstance() const
{
    return new PlayMp3Command;
}

helloworld::Announcement::Announcement(const ::std::string& __ice_message) :
    message(__ice_message)
{
}

::Ice::ObjectPtr
helloworld::Announcement::ice_clone() const
{
    ::helloworld::AnnouncementPtr __p = new ::helloworld::Announcement(*this);
    return __p;
}

static const ::std::string __helloworld__Announcement_ids[2] =
{
    "::Ice::Object",
    "::helloworld::Announcement"
};

bool
helloworld::Announcement::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__helloworld__Announcement_ids, __helloworld__Announcement_ids + 2, _s);
}

::std::vector< ::std::string>
helloworld::Announcement::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__helloworld__Announcement_ids[0], &__helloworld__Announcement_ids[2]);
}

const ::std::string&
helloworld::Announcement::ice_id(const ::Ice::Current&) const
{
    return __helloworld__Announcement_ids[1];
}

const ::std::string&
helloworld::Announcement::ice_staticId()
{
    return __helloworld__Announcement_ids[1];
}

void
helloworld::Announcement::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(message);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
helloworld::Announcement::__read(::IceInternal::BasicStream* __is, bool __rid)
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
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
helloworld::Announcement::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(message);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
helloworld::Announcement::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    message = __inS->readString();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__helloworld__Announcement : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::helloworld::Announcement::ice_staticId());
        return new ::helloworld::Announcement;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__helloworld__Announcement_Ptr = new __F__helloworld__Announcement;

const ::Ice::ObjectFactoryPtr&
helloworld::Announcement::ice_factory()
{
    return __F__helloworld__Announcement_Ptr;
}

class __F__helloworld__Announcement__Init
{
public:

    __F__helloworld__Announcement__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::helloworld::Announcement::ice_staticId(), ::helloworld::Announcement::ice_factory());
    }

    ~__F__helloworld__Announcement__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::helloworld::Announcement::ice_staticId());
    }
};

static __F__helloworld__Announcement__Init __F__helloworld__Announcement__i;

#ifdef __APPLE__
extern "C" { void __F__helloworld__Announcement__initializer() {} }
#endif

void 
helloworld::__patch__AnnouncementPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::helloworld::AnnouncementPtr* p = static_cast< ::helloworld::AnnouncementPtr*>(__addr);
    assert(p);
    *p = ::helloworld::AnnouncementPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::helloworld::Announcement::ice_staticId(), v->ice_id());
    }
}

bool
helloworld::operator==(const ::helloworld::Announcement& l, const ::helloworld::Announcement& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
helloworld::operator<(const ::helloworld::Announcement& l, const ::helloworld::Announcement& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

helloworld::MaryCommand::MaryCommand(const ::std::string& __ice_text) :
    text(__ice_text)
{
}

::Ice::ObjectPtr
helloworld::MaryCommand::ice_clone() const
{
    ::helloworld::MaryCommandPtr __p = new ::helloworld::MaryCommand(*this);
    return __p;
}

static const ::std::string __helloworld__MaryCommand_ids[2] =
{
    "::Ice::Object",
    "::helloworld::MaryCommand"
};

bool
helloworld::MaryCommand::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__helloworld__MaryCommand_ids, __helloworld__MaryCommand_ids + 2, _s);
}

::std::vector< ::std::string>
helloworld::MaryCommand::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__helloworld__MaryCommand_ids[0], &__helloworld__MaryCommand_ids[2]);
}

const ::std::string&
helloworld::MaryCommand::ice_id(const ::Ice::Current&) const
{
    return __helloworld__MaryCommand_ids[1];
}

const ::std::string&
helloworld::MaryCommand::ice_staticId()
{
    return __helloworld__MaryCommand_ids[1];
}

void
helloworld::MaryCommand::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(text);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
helloworld::MaryCommand::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(text);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
helloworld::MaryCommand::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(text);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
helloworld::MaryCommand::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    text = __inS->readString();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__helloworld__MaryCommand : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::helloworld::MaryCommand::ice_staticId());
        return new ::helloworld::MaryCommand;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__helloworld__MaryCommand_Ptr = new __F__helloworld__MaryCommand;

const ::Ice::ObjectFactoryPtr&
helloworld::MaryCommand::ice_factory()
{
    return __F__helloworld__MaryCommand_Ptr;
}

class __F__helloworld__MaryCommand__Init
{
public:

    __F__helloworld__MaryCommand__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::helloworld::MaryCommand::ice_staticId(), ::helloworld::MaryCommand::ice_factory());
    }

    ~__F__helloworld__MaryCommand__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::helloworld::MaryCommand::ice_staticId());
    }
};

static __F__helloworld__MaryCommand__Init __F__helloworld__MaryCommand__i;

#ifdef __APPLE__
extern "C" { void __F__helloworld__MaryCommand__initializer() {} }
#endif

void 
helloworld::__patch__MaryCommandPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::helloworld::MaryCommandPtr* p = static_cast< ::helloworld::MaryCommandPtr*>(__addr);
    assert(p);
    *p = ::helloworld::MaryCommandPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::helloworld::MaryCommand::ice_staticId(), v->ice_id());
    }
}

bool
helloworld::operator==(const ::helloworld::MaryCommand& l, const ::helloworld::MaryCommand& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
helloworld::operator<(const ::helloworld::MaryCommand& l, const ::helloworld::MaryCommand& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

helloworld::PlayMp3Command::PlayMp3Command(::Ice::Long __ice_song) :
    song(__ice_song)
{
}

::Ice::ObjectPtr
helloworld::PlayMp3Command::ice_clone() const
{
    ::helloworld::PlayMp3CommandPtr __p = new ::helloworld::PlayMp3Command(*this);
    return __p;
}

static const ::std::string __helloworld__PlayMp3Command_ids[2] =
{
    "::Ice::Object",
    "::helloworld::PlayMp3Command"
};

bool
helloworld::PlayMp3Command::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__helloworld__PlayMp3Command_ids, __helloworld__PlayMp3Command_ids + 2, _s);
}

::std::vector< ::std::string>
helloworld::PlayMp3Command::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__helloworld__PlayMp3Command_ids[0], &__helloworld__PlayMp3Command_ids[2]);
}

const ::std::string&
helloworld::PlayMp3Command::ice_id(const ::Ice::Current&) const
{
    return __helloworld__PlayMp3Command_ids[1];
}

const ::std::string&
helloworld::PlayMp3Command::ice_staticId()
{
    return __helloworld__PlayMp3Command_ids[1];
}

void
helloworld::PlayMp3Command::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(song);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
helloworld::PlayMp3Command::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(song);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
helloworld::PlayMp3Command::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeLong(song);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
helloworld::PlayMp3Command::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    song = __inS->readLong();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__helloworld__PlayMp3Command : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::helloworld::PlayMp3Command::ice_staticId());
        return new ::helloworld::PlayMp3Command;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__helloworld__PlayMp3Command_Ptr = new __F__helloworld__PlayMp3Command;

const ::Ice::ObjectFactoryPtr&
helloworld::PlayMp3Command::ice_factory()
{
    return __F__helloworld__PlayMp3Command_Ptr;
}

class __F__helloworld__PlayMp3Command__Init
{
public:

    __F__helloworld__PlayMp3Command__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::helloworld::PlayMp3Command::ice_staticId(), ::helloworld::PlayMp3Command::ice_factory());
    }

    ~__F__helloworld__PlayMp3Command__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::helloworld::PlayMp3Command::ice_staticId());
    }
};

static __F__helloworld__PlayMp3Command__Init __F__helloworld__PlayMp3Command__i;

#ifdef __APPLE__
extern "C" { void __F__helloworld__PlayMp3Command__initializer() {} }
#endif

void 
helloworld::__patch__PlayMp3CommandPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::helloworld::PlayMp3CommandPtr* p = static_cast< ::helloworld::PlayMp3CommandPtr*>(__addr);
    assert(p);
    *p = ::helloworld::PlayMp3CommandPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::helloworld::PlayMp3Command::ice_staticId(), v->ice_id());
    }
}

bool
helloworld::operator==(const ::helloworld::PlayMp3Command& l, const ::helloworld::PlayMp3Command& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
helloworld::operator<(const ::helloworld::PlayMp3Command& l, const ::helloworld::PlayMp3Command& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
