// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `ComedyEssentials.ice'

#ifndef ___home_plison_svn_cogx_binding_development_tools_cast_src_c___cast_examples_comedyarch_ComedyEssentials_hpp__
#define ___home_plison_svn_cogx_binding_development_tools_cast_src_c___cast_examples_comedyarch_ComedyEssentials_hpp__

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

namespace comedyarch
{

namespace autogen
{

class Joke;

class JokeBook;

class DirectorAction;

class Reaction;

class TheGreatPretender;

}

}

}

namespace comedyarch
{

namespace autogen
{

class Joke;
bool operator==(const Joke&, const Joke&);
bool operator<(const Joke&, const Joke&);

class JokeBook;
bool operator==(const JokeBook&, const JokeBook&);
bool operator<(const JokeBook&, const JokeBook&);

class DirectorAction;
bool operator==(const DirectorAction&, const DirectorAction&);
bool operator<(const DirectorAction&, const DirectorAction&);

class Reaction;
bool operator==(const Reaction&, const Reaction&);
bool operator<(const Reaction&, const Reaction&);

class TheGreatPretender;
bool operator==(const TheGreatPretender&, const TheGreatPretender&);
bool operator<(const TheGreatPretender&, const TheGreatPretender&);

}

}

namespace IceInternal
{

::Ice::Object* upCast(::comedyarch::autogen::Joke*);
::IceProxy::Ice::Object* upCast(::IceProxy::comedyarch::autogen::Joke*);

::Ice::Object* upCast(::comedyarch::autogen::JokeBook*);
::IceProxy::Ice::Object* upCast(::IceProxy::comedyarch::autogen::JokeBook*);

::Ice::Object* upCast(::comedyarch::autogen::DirectorAction*);
::IceProxy::Ice::Object* upCast(::IceProxy::comedyarch::autogen::DirectorAction*);

::Ice::Object* upCast(::comedyarch::autogen::Reaction*);
::IceProxy::Ice::Object* upCast(::IceProxy::comedyarch::autogen::Reaction*);

::Ice::Object* upCast(::comedyarch::autogen::TheGreatPretender*);
::IceProxy::Ice::Object* upCast(::IceProxy::comedyarch::autogen::TheGreatPretender*);

}

namespace comedyarch
{

namespace autogen
{

typedef ::IceInternal::Handle< ::comedyarch::autogen::Joke> JokePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::comedyarch::autogen::Joke> JokePrx;

void __read(::IceInternal::BasicStream*, JokePrx&);
void __patch__JokePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::comedyarch::autogen::JokeBook> JokeBookPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::comedyarch::autogen::JokeBook> JokeBookPrx;

void __read(::IceInternal::BasicStream*, JokeBookPrx&);
void __patch__JokeBookPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::comedyarch::autogen::DirectorAction> DirectorActionPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::comedyarch::autogen::DirectorAction> DirectorActionPrx;

void __read(::IceInternal::BasicStream*, DirectorActionPrx&);
void __patch__DirectorActionPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::comedyarch::autogen::Reaction> ReactionPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::comedyarch::autogen::Reaction> ReactionPrx;

void __read(::IceInternal::BasicStream*, ReactionPrx&);
void __patch__ReactionPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::comedyarch::autogen::TheGreatPretender> TheGreatPretenderPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::comedyarch::autogen::TheGreatPretender> TheGreatPretenderPrx;

void __read(::IceInternal::BasicStream*, TheGreatPretenderPrx&);
void __patch__TheGreatPretenderPtr(void*, ::Ice::ObjectPtr&);

}

}

namespace comedyarch
{

namespace autogen
{

typedef ::std::vector< ::std::string> StringSeq;

enum Funny
{
    HAHA,
    PECULIAR
};

void __write(::IceInternal::BasicStream*, Funny);
void __read(::IceInternal::BasicStream*, Funny&);

typedef ::std::vector< ::comedyarch::autogen::JokePtr> JokeList;
void __writeJokeList(::IceInternal::BasicStream*, const ::comedyarch::autogen::JokePtr*, const ::comedyarch::autogen::JokePtr*);
void __readJokeList(::IceInternal::BasicStream*, JokeList&);

enum DirectorActionType
{
    AskTheAudience,
    CheckTheReaction
};

void __write(::IceInternal::BasicStream*, DirectorActionType);
void __read(::IceInternal::BasicStream*, DirectorActionType&);

}

}

namespace IceProxy
{

namespace comedyarch
{

namespace autogen
{

class Joke : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Joke> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Joke> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Joke*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Joke*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class JokeBook : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<JokeBook> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<JokeBook> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<JokeBook*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<JokeBook*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class DirectorAction : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<DirectorAction> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DirectorAction*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<DirectorAction*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Reaction : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Reaction> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Reaction> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Reaction*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Reaction*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class TheGreatPretender : virtual public ::IceProxy::Ice::Object
{
public:

    void getLies()
    {
        getLies(0);
    }
    void getLies(const ::Ice::Context& __ctx)
    {
        getLies(&__ctx);
    }
    
private:

    void getLies(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TheGreatPretender> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TheGreatPretender*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TheGreatPretender*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

}

namespace IceDelegate
{

namespace comedyarch
{

namespace autogen
{

class Joke : virtual public ::IceDelegate::Ice::Object
{
public:
};

class JokeBook : virtual public ::IceDelegate::Ice::Object
{
public:
};

class DirectorAction : virtual public ::IceDelegate::Ice::Object
{
public:
};

class Reaction : virtual public ::IceDelegate::Ice::Object
{
public:
};

class TheGreatPretender : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void getLies(const ::Ice::Context*) = 0;
};

}

}

}

namespace IceDelegateM
{

namespace comedyarch
{

namespace autogen
{

class Joke : virtual public ::IceDelegate::comedyarch::autogen::Joke,
             virtual public ::IceDelegateM::Ice::Object
{
public:
};

class JokeBook : virtual public ::IceDelegate::comedyarch::autogen::JokeBook,
                 virtual public ::IceDelegateM::Ice::Object
{
public:
};

class DirectorAction : virtual public ::IceDelegate::comedyarch::autogen::DirectorAction,
                       virtual public ::IceDelegateM::Ice::Object
{
public:
};

class Reaction : virtual public ::IceDelegate::comedyarch::autogen::Reaction,
                 virtual public ::IceDelegateM::Ice::Object
{
public:
};

class TheGreatPretender : virtual public ::IceDelegate::comedyarch::autogen::TheGreatPretender,
                          virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void getLies(const ::Ice::Context*);
};

}

}

}

namespace IceDelegateD
{

namespace comedyarch
{

namespace autogen
{

class Joke : virtual public ::IceDelegate::comedyarch::autogen::Joke,
             virtual public ::IceDelegateD::Ice::Object
{
public:
};

class JokeBook : virtual public ::IceDelegate::comedyarch::autogen::JokeBook,
                 virtual public ::IceDelegateD::Ice::Object
{
public:
};

class DirectorAction : virtual public ::IceDelegate::comedyarch::autogen::DirectorAction,
                       virtual public ::IceDelegateD::Ice::Object
{
public:
};

class Reaction : virtual public ::IceDelegate::comedyarch::autogen::Reaction,
                 virtual public ::IceDelegateD::Ice::Object
{
public:
};

class TheGreatPretender : virtual public ::IceDelegate::comedyarch::autogen::TheGreatPretender,
                          virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void getLies(const ::Ice::Context*);
};

}

}

}

namespace comedyarch
{

namespace autogen
{

class Joke : virtual public ::Ice::Object
{
public:

    typedef JokePrx ProxyType;
    typedef JokePtr PointerType;
    
    Joke() {}
    Joke(const ::std::string&, const ::std::string&);
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

    virtual ~Joke() {}

    friend class Joke__staticInit;

public:

    ::std::string setup;

    ::std::string punchline;
};

class Joke__staticInit
{
public:

    ::comedyarch::autogen::Joke _init;
};

static ::comedyarch::autogen::Joke__staticInit _Joke_init;

class JokeBook : virtual public ::Ice::Object
{
public:

    typedef JokeBookPrx ProxyType;
    typedef JokeBookPtr PointerType;
    
    JokeBook() {}
    JokeBook(const ::std::string&, ::Ice::Long, const ::comedyarch::autogen::JokeList&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void __incRef();
    virtual void __decRef();
    virtual void __addObject(::IceInternal::GCCountMap&);
    virtual bool __usesClasses();
    virtual void __gcReachable(::IceInternal::GCCountMap&) const;
    virtual void __gcClear();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~JokeBook() {}

public:

    ::std::string title;

    ::Ice::Long jokeCount;

    ::comedyarch::autogen::JokeList jokes;
};

class DirectorAction : virtual public ::Ice::Object
{
public:

    typedef DirectorActionPrx ProxyType;
    typedef DirectorActionPtr PointerType;
    
    DirectorAction() {}
    DirectorAction(::comedyarch::autogen::DirectorActionType, const ::cast::cdl::WorkingMemoryAddress&);
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

    virtual ~DirectorAction() {}

public:

    ::comedyarch::autogen::DirectorActionType action;

    ::cast::cdl::WorkingMemoryAddress address;
};

class Reaction : virtual public ::Ice::Object
{
public:

    typedef ReactionPrx ProxyType;
    typedef ReactionPtr PointerType;
    
    Reaction() {}
    explicit Reaction(const ::std::string&);
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

    virtual ~Reaction() {}

public:

    ::std::string react;
};

class TheGreatPretender : virtual public ::Ice::Object
{
public:

    typedef TheGreatPretenderPrx ProxyType;
    typedef TheGreatPretenderPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void getLies(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getLies(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

}

#endif
