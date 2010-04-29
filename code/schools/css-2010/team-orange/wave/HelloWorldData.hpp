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

#ifndef ___home_sahip_tutorial_subarchitectures_hello_world_src_c___HelloWorldData_hpp__
#define ___home_sahip_tutorial_subarchitectures_hello_world_src_c___HelloWorldData_hpp__

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
#include <Ice/FactoryTable.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

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

namespace IceProxy
{

namespace helloworld
{

class Announcement;

class MaryCommand;

class PlayMp3Command;

}

}

namespace helloworld
{

class Announcement;
bool operator==(const Announcement&, const Announcement&);
bool operator<(const Announcement&, const Announcement&);

class MaryCommand;
bool operator==(const MaryCommand&, const MaryCommand&);
bool operator<(const MaryCommand&, const MaryCommand&);

class PlayMp3Command;
bool operator==(const PlayMp3Command&, const PlayMp3Command&);
bool operator<(const PlayMp3Command&, const PlayMp3Command&);

}

namespace IceInternal
{

::Ice::Object* upCast(::helloworld::Announcement*);
::IceProxy::Ice::Object* upCast(::IceProxy::helloworld::Announcement*);

::Ice::Object* upCast(::helloworld::MaryCommand*);
::IceProxy::Ice::Object* upCast(::IceProxy::helloworld::MaryCommand*);

::Ice::Object* upCast(::helloworld::PlayMp3Command*);
::IceProxy::Ice::Object* upCast(::IceProxy::helloworld::PlayMp3Command*);

}

namespace helloworld
{

typedef ::IceInternal::Handle< ::helloworld::Announcement> AnnouncementPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::helloworld::Announcement> AnnouncementPrx;

void __read(::IceInternal::BasicStream*, AnnouncementPrx&);
void __patch__AnnouncementPtr(void*, ::Ice::ObjectPtr&);

void ice_writeAnnouncementPrx(const ::Ice::OutputStreamPtr&, const AnnouncementPrx&);
void ice_readAnnouncementPrx(const ::Ice::InputStreamPtr&, AnnouncementPrx&);
void ice_writeAnnouncement(const ::Ice::OutputStreamPtr&, const AnnouncementPtr&);
void ice_readAnnouncement(const ::Ice::InputStreamPtr&, AnnouncementPtr&);

typedef ::IceInternal::Handle< ::helloworld::MaryCommand> MaryCommandPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::helloworld::MaryCommand> MaryCommandPrx;

void __read(::IceInternal::BasicStream*, MaryCommandPrx&);
void __patch__MaryCommandPtr(void*, ::Ice::ObjectPtr&);

void ice_writeMaryCommandPrx(const ::Ice::OutputStreamPtr&, const MaryCommandPrx&);
void ice_readMaryCommandPrx(const ::Ice::InputStreamPtr&, MaryCommandPrx&);
void ice_writeMaryCommand(const ::Ice::OutputStreamPtr&, const MaryCommandPtr&);
void ice_readMaryCommand(const ::Ice::InputStreamPtr&, MaryCommandPtr&);

typedef ::IceInternal::Handle< ::helloworld::PlayMp3Command> PlayMp3CommandPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::helloworld::PlayMp3Command> PlayMp3CommandPrx;

void __read(::IceInternal::BasicStream*, PlayMp3CommandPrx&);
void __patch__PlayMp3CommandPtr(void*, ::Ice::ObjectPtr&);

void ice_writePlayMp3CommandPrx(const ::Ice::OutputStreamPtr&, const PlayMp3CommandPrx&);
void ice_readPlayMp3CommandPrx(const ::Ice::InputStreamPtr&, PlayMp3CommandPrx&);
void ice_writePlayMp3Command(const ::Ice::OutputStreamPtr&, const PlayMp3CommandPtr&);
void ice_readPlayMp3Command(const ::Ice::InputStreamPtr&, PlayMp3CommandPtr&);

}

namespace IceProxy
{

namespace helloworld
{

class Announcement : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Announcement> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Announcement> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Announcement*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Announcement*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class MaryCommand : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<MaryCommand> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MaryCommand*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<MaryCommand*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class PlayMp3Command : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PlayMp3Command> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PlayMp3Command*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<PlayMp3Command*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace helloworld
{

class Announcement : virtual public ::IceDelegate::Ice::Object
{
public:
};

class MaryCommand : virtual public ::IceDelegate::Ice::Object
{
public:
};

class PlayMp3Command : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

}

namespace IceDelegateM
{

namespace helloworld
{

class Announcement : virtual public ::IceDelegate::helloworld::Announcement,
                     virtual public ::IceDelegateM::Ice::Object
{
public:
};

class MaryCommand : virtual public ::IceDelegate::helloworld::MaryCommand,
                    virtual public ::IceDelegateM::Ice::Object
{
public:
};

class PlayMp3Command : virtual public ::IceDelegate::helloworld::PlayMp3Command,
                       virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

}

namespace IceDelegateD
{

namespace helloworld
{

class Announcement : virtual public ::IceDelegate::helloworld::Announcement,
                     virtual public ::IceDelegateD::Ice::Object
{
public:
};

class MaryCommand : virtual public ::IceDelegate::helloworld::MaryCommand,
                    virtual public ::IceDelegateD::Ice::Object
{
public:
};

class PlayMp3Command : virtual public ::IceDelegate::helloworld::PlayMp3Command,
                       virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

}

namespace helloworld
{

class Announcement : virtual public ::Ice::Object
{
public:

    typedef AnnouncementPrx ProxyType;
    typedef AnnouncementPtr PointerType;
    
    Announcement() {}
    explicit Announcement(const ::std::string&);
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

    virtual ~Announcement() {}

    friend class Announcement__staticInit;

public:

    ::std::string message;
};

class Announcement__staticInit
{
public:

    ::helloworld::Announcement _init;
};

static Announcement__staticInit _Announcement_init;

class MaryCommand : virtual public ::Ice::Object
{
public:

    typedef MaryCommandPrx ProxyType;
    typedef MaryCommandPtr PointerType;
    
    MaryCommand() {}
    explicit MaryCommand(const ::std::string&);
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

    virtual ~MaryCommand() {}

public:

    ::std::string text;
};

class PlayMp3Command : virtual public ::Ice::Object
{
public:

    typedef PlayMp3CommandPrx ProxyType;
    typedef PlayMp3CommandPtr PointerType;
    
    PlayMp3Command() {}
    explicit PlayMp3Command(::Ice::Long);
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

    virtual ~PlayMp3Command() {}

public:

    ::Ice::Long song;
};

}

#endif
