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

#include <ComedyEssentials.hpp>
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

static const ::std::string __comedyarch__autogen__TheGreatPretender__getLies_name = "getLies";

::Ice::Object* IceInternal::upCast(::comedyarch::autogen::Joke* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::comedyarch::autogen::Joke* p) { return p; }

::Ice::Object* IceInternal::upCast(::comedyarch::autogen::JokeBook* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::comedyarch::autogen::JokeBook* p) { return p; }

::Ice::Object* IceInternal::upCast(::comedyarch::autogen::DirectorAction* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::comedyarch::autogen::DirectorAction* p) { return p; }

::Ice::Object* IceInternal::upCast(::comedyarch::autogen::Reaction* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::comedyarch::autogen::Reaction* p) { return p; }

::Ice::Object* IceInternal::upCast(::comedyarch::autogen::TheGreatPretender* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::comedyarch::autogen::TheGreatPretender* p) { return p; }

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::JokePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::comedyarch::autogen::Joke;
        v->__copyFrom(proxy);
    }
}

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::JokeBookPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::comedyarch::autogen::JokeBook;
        v->__copyFrom(proxy);
    }
}

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::DirectorActionPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::comedyarch::autogen::DirectorAction;
        v->__copyFrom(proxy);
    }
}

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::ReactionPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::comedyarch::autogen::Reaction;
        v->__copyFrom(proxy);
    }
}

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::TheGreatPretenderPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::comedyarch::autogen::TheGreatPretender;
        v->__copyFrom(proxy);
    }
}

void
comedyarch::autogen::__write(::IceInternal::BasicStream* __os, ::comedyarch::autogen::Funny v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::Funny& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::comedyarch::autogen::Funny>(val);
}

void
comedyarch::autogen::__writeJokeList(::IceInternal::BasicStream* __os, const ::comedyarch::autogen::JokePtr* begin, const ::comedyarch::autogen::JokePtr* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(begin[i].get())));
    }
}

void
comedyarch::autogen::__readJokeList(::IceInternal::BasicStream* __is, ::comedyarch::autogen::JokeList& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 4);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        __is->read(::comedyarch::autogen::__patch__JokePtr, &v[i]);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

void
comedyarch::autogen::__write(::IceInternal::BasicStream* __os, ::comedyarch::autogen::DirectorActionType v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
comedyarch::autogen::__read(::IceInternal::BasicStream* __is, ::comedyarch::autogen::DirectorActionType& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::comedyarch::autogen::DirectorActionType>(val);
}

const ::std::string&
IceProxy::comedyarch::autogen::Joke::ice_staticId()
{
    return ::comedyarch::autogen::Joke::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::comedyarch::autogen::Joke::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::comedyarch::autogen::Joke);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::comedyarch::autogen::Joke::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::comedyarch::autogen::Joke);
}

::IceProxy::Ice::Object*
IceProxy::comedyarch::autogen::Joke::__newInstance() const
{
    return new Joke;
}

const ::std::string&
IceProxy::comedyarch::autogen::JokeBook::ice_staticId()
{
    return ::comedyarch::autogen::JokeBook::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::comedyarch::autogen::JokeBook::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::comedyarch::autogen::JokeBook);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::comedyarch::autogen::JokeBook::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::comedyarch::autogen::JokeBook);
}

::IceProxy::Ice::Object*
IceProxy::comedyarch::autogen::JokeBook::__newInstance() const
{
    return new JokeBook;
}

const ::std::string&
IceProxy::comedyarch::autogen::DirectorAction::ice_staticId()
{
    return ::comedyarch::autogen::DirectorAction::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::comedyarch::autogen::DirectorAction::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::comedyarch::autogen::DirectorAction);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::comedyarch::autogen::DirectorAction::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::comedyarch::autogen::DirectorAction);
}

::IceProxy::Ice::Object*
IceProxy::comedyarch::autogen::DirectorAction::__newInstance() const
{
    return new DirectorAction;
}

const ::std::string&
IceProxy::comedyarch::autogen::Reaction::ice_staticId()
{
    return ::comedyarch::autogen::Reaction::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::comedyarch::autogen::Reaction::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::comedyarch::autogen::Reaction);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::comedyarch::autogen::Reaction::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::comedyarch::autogen::Reaction);
}

::IceProxy::Ice::Object*
IceProxy::comedyarch::autogen::Reaction::__newInstance() const
{
    return new Reaction;
}

void
IceProxy::comedyarch::autogen::TheGreatPretender::getLies(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::comedyarch::autogen::TheGreatPretender* __del = dynamic_cast< ::IceDelegate::comedyarch::autogen::TheGreatPretender*>(__delBase.get());
            __del->getLies(__ctx);
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
IceProxy::comedyarch::autogen::TheGreatPretender::ice_staticId()
{
    return ::comedyarch::autogen::TheGreatPretender::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::comedyarch::autogen::TheGreatPretender::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::comedyarch::autogen::TheGreatPretender);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::comedyarch::autogen::TheGreatPretender::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::comedyarch::autogen::TheGreatPretender);
}

::IceProxy::Ice::Object*
IceProxy::comedyarch::autogen::TheGreatPretender::__newInstance() const
{
    return new TheGreatPretender;
}

void
IceDelegateM::comedyarch::autogen::TheGreatPretender::getLies(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __comedyarch__autogen__TheGreatPretender__getLies_name, ::Ice::Normal, __context);
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
IceDelegateD::comedyarch::autogen::TheGreatPretender::getLies(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::comedyarch::autogen::TheGreatPretender* servant = dynamic_cast< ::comedyarch::autogen::TheGreatPretender*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getLies(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __comedyarch__autogen__TheGreatPretender__getLies_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
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

comedyarch::autogen::Joke::Joke(const ::std::string& __ice_setup, const ::std::string& __ice_punchline) :
    setup(__ice_setup),
    punchline(__ice_punchline)
{
}

::Ice::ObjectPtr
comedyarch::autogen::Joke::ice_clone() const
{
    ::comedyarch::autogen::JokePtr __p = new ::comedyarch::autogen::Joke(*this);
    return __p;
}

static const ::std::string __comedyarch__autogen__Joke_ids[2] =
{
    "::Ice::Object",
    "::comedyarch::autogen::Joke"
};

bool
comedyarch::autogen::Joke::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__comedyarch__autogen__Joke_ids, __comedyarch__autogen__Joke_ids + 2, _s);
}

::std::vector< ::std::string>
comedyarch::autogen::Joke::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__comedyarch__autogen__Joke_ids[0], &__comedyarch__autogen__Joke_ids[2]);
}

const ::std::string&
comedyarch::autogen::Joke::ice_id(const ::Ice::Current&) const
{
    return __comedyarch__autogen__Joke_ids[1];
}

const ::std::string&
comedyarch::autogen::Joke::ice_staticId()
{
    return __comedyarch__autogen__Joke_ids[1];
}

void
comedyarch::autogen::Joke::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(setup);
    __os->write(punchline);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
comedyarch::autogen::Joke::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(setup);
    __is->read(punchline);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
comedyarch::autogen::Joke::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::Joke was not generated with stream support";
    throw ex;
}

void
comedyarch::autogen::Joke::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::Joke was not generated with stream support";
    throw ex;
}

class __F__comedyarch__autogen__Joke : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::comedyarch::autogen::Joke::ice_staticId());
        return new ::comedyarch::autogen::Joke;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__comedyarch__autogen__Joke_Ptr = new __F__comedyarch__autogen__Joke;

const ::Ice::ObjectFactoryPtr&
comedyarch::autogen::Joke::ice_factory()
{
    return __F__comedyarch__autogen__Joke_Ptr;
}

class __F__comedyarch__autogen__Joke__Init
{
public:

    __F__comedyarch__autogen__Joke__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::comedyarch::autogen::Joke::ice_staticId(), ::comedyarch::autogen::Joke::ice_factory());
    }

    ~__F__comedyarch__autogen__Joke__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::comedyarch::autogen::Joke::ice_staticId());
    }
};

static __F__comedyarch__autogen__Joke__Init __F__comedyarch__autogen__Joke__i;

#ifdef __APPLE__
extern "C" { void __F__comedyarch__autogen__Joke__initializer() {} }
#endif

void 
comedyarch::autogen::__patch__JokePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::comedyarch::autogen::JokePtr* p = static_cast< ::comedyarch::autogen::JokePtr*>(__addr);
    assert(p);
    *p = ::comedyarch::autogen::JokePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::comedyarch::autogen::Joke::ice_staticId(), v->ice_id());
    }
}

bool
comedyarch::autogen::operator==(const ::comedyarch::autogen::Joke& l, const ::comedyarch::autogen::Joke& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
comedyarch::autogen::operator<(const ::comedyarch::autogen::Joke& l, const ::comedyarch::autogen::Joke& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

comedyarch::autogen::JokeBook::JokeBook(const ::std::string& __ice_title, ::Ice::Long __ice_jokeCount, const ::comedyarch::autogen::JokeList& __ice_jokes) :
    title(__ice_title),
    jokeCount(__ice_jokeCount),
    jokes(__ice_jokes)
{
}

::Ice::ObjectPtr
comedyarch::autogen::JokeBook::ice_clone() const
{
    ::comedyarch::autogen::JokeBookPtr __p = new ::comedyarch::autogen::JokeBook(*this);
    return __p;
}

static const ::std::string __comedyarch__autogen__JokeBook_ids[2] =
{
    "::Ice::Object",
    "::comedyarch::autogen::JokeBook"
};

bool
comedyarch::autogen::JokeBook::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__comedyarch__autogen__JokeBook_ids, __comedyarch__autogen__JokeBook_ids + 2, _s);
}

::std::vector< ::std::string>
comedyarch::autogen::JokeBook::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__comedyarch__autogen__JokeBook_ids[0], &__comedyarch__autogen__JokeBook_ids[2]);
}

const ::std::string&
comedyarch::autogen::JokeBook::ice_id(const ::Ice::Current&) const
{
    return __comedyarch__autogen__JokeBook_ids[1];
}

const ::std::string&
comedyarch::autogen::JokeBook::ice_staticId()
{
    return __comedyarch__autogen__JokeBook_ids[1];
}

void
comedyarch::autogen::JokeBook::__incRef()
{
    __gcIncRef();
}

void
comedyarch::autogen::JokeBook::__decRef()
{
    __gcDecRef();
}

void
comedyarch::autogen::JokeBook::__addObject(::IceInternal::GCCountMap& _c)
{
    ::IceInternal::GCCountMap::iterator pos = _c.find(this);
    if(pos == _c.end())
    {
        _c[this] = 1;
    }
    else
    {
        ++pos->second;
    }
}

bool
comedyarch::autogen::JokeBook::__usesClasses()
{
    return true;
}

void
comedyarch::autogen::JokeBook::__gcReachable(::IceInternal::GCCountMap& _c) const
{
    {
        for(::comedyarch::autogen::JokeList::const_iterator _i0 = jokes.begin(); _i0 != jokes.end(); ++_i0)
        {
            if((*_i0))
            {
                ::IceInternal::upCast((*_i0).get())->__addObject(_c);
            }
        }
    }
}

void
comedyarch::autogen::JokeBook::__gcClear()
{
    {
        for(::comedyarch::autogen::JokeList::iterator _i0 = jokes.begin(); _i0 != jokes.end(); ++_i0)
        {
            if((*_i0))
            {
                if(::IceInternal::upCast((*_i0).get())->__usesClasses())
                {
                    ::IceInternal::upCast((*_i0).get())->__decRefUnsafe();
                    (*_i0).__clearHandleUnsafe();
                }
                else
                {
                    (*_i0) = 0;
                }
            }
        }
    }
}

void
comedyarch::autogen::JokeBook::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(title);
    __os->write(jokeCount);
    if(jokes.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::comedyarch::autogen::__writeJokeList(__os, &jokes[0], &jokes[0] + jokes.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
comedyarch::autogen::JokeBook::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(title);
    __is->read(jokeCount);
    ::comedyarch::autogen::__readJokeList(__is, jokes);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
comedyarch::autogen::JokeBook::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::JokeBook was not generated with stream support";
    throw ex;
}

void
comedyarch::autogen::JokeBook::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::JokeBook was not generated with stream support";
    throw ex;
}

class __F__comedyarch__autogen__JokeBook : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::comedyarch::autogen::JokeBook::ice_staticId());
        return new ::comedyarch::autogen::JokeBook;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__comedyarch__autogen__JokeBook_Ptr = new __F__comedyarch__autogen__JokeBook;

const ::Ice::ObjectFactoryPtr&
comedyarch::autogen::JokeBook::ice_factory()
{
    return __F__comedyarch__autogen__JokeBook_Ptr;
}

class __F__comedyarch__autogen__JokeBook__Init
{
public:

    __F__comedyarch__autogen__JokeBook__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::comedyarch::autogen::JokeBook::ice_staticId(), ::comedyarch::autogen::JokeBook::ice_factory());
    }

    ~__F__comedyarch__autogen__JokeBook__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::comedyarch::autogen::JokeBook::ice_staticId());
    }
};

static __F__comedyarch__autogen__JokeBook__Init __F__comedyarch__autogen__JokeBook__i;

#ifdef __APPLE__
extern "C" { void __F__comedyarch__autogen__JokeBook__initializer() {} }
#endif

void 
comedyarch::autogen::__patch__JokeBookPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::comedyarch::autogen::JokeBookPtr* p = static_cast< ::comedyarch::autogen::JokeBookPtr*>(__addr);
    assert(p);
    *p = ::comedyarch::autogen::JokeBookPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::comedyarch::autogen::JokeBook::ice_staticId(), v->ice_id());
    }
}

bool
comedyarch::autogen::operator==(const ::comedyarch::autogen::JokeBook& l, const ::comedyarch::autogen::JokeBook& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
comedyarch::autogen::operator<(const ::comedyarch::autogen::JokeBook& l, const ::comedyarch::autogen::JokeBook& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

comedyarch::autogen::DirectorAction::DirectorAction(::comedyarch::autogen::DirectorActionType __ice_action, const ::cast::cdl::WorkingMemoryAddress& __ice_address) :
    action(__ice_action),
    address(__ice_address)
{
}

::Ice::ObjectPtr
comedyarch::autogen::DirectorAction::ice_clone() const
{
    ::comedyarch::autogen::DirectorActionPtr __p = new ::comedyarch::autogen::DirectorAction(*this);
    return __p;
}

static const ::std::string __comedyarch__autogen__DirectorAction_ids[2] =
{
    "::Ice::Object",
    "::comedyarch::autogen::DirectorAction"
};

bool
comedyarch::autogen::DirectorAction::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__comedyarch__autogen__DirectorAction_ids, __comedyarch__autogen__DirectorAction_ids + 2, _s);
}

::std::vector< ::std::string>
comedyarch::autogen::DirectorAction::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__comedyarch__autogen__DirectorAction_ids[0], &__comedyarch__autogen__DirectorAction_ids[2]);
}

const ::std::string&
comedyarch::autogen::DirectorAction::ice_id(const ::Ice::Current&) const
{
    return __comedyarch__autogen__DirectorAction_ids[1];
}

const ::std::string&
comedyarch::autogen::DirectorAction::ice_staticId()
{
    return __comedyarch__autogen__DirectorAction_ids[1];
}

void
comedyarch::autogen::DirectorAction::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    ::comedyarch::autogen::__write(__os, action);
    address.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
comedyarch::autogen::DirectorAction::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    ::comedyarch::autogen::__read(__is, action);
    address.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
comedyarch::autogen::DirectorAction::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::DirectorAction was not generated with stream support";
    throw ex;
}

void
comedyarch::autogen::DirectorAction::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::DirectorAction was not generated with stream support";
    throw ex;
}

class __F__comedyarch__autogen__DirectorAction : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::comedyarch::autogen::DirectorAction::ice_staticId());
        return new ::comedyarch::autogen::DirectorAction;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__comedyarch__autogen__DirectorAction_Ptr = new __F__comedyarch__autogen__DirectorAction;

const ::Ice::ObjectFactoryPtr&
comedyarch::autogen::DirectorAction::ice_factory()
{
    return __F__comedyarch__autogen__DirectorAction_Ptr;
}

class __F__comedyarch__autogen__DirectorAction__Init
{
public:

    __F__comedyarch__autogen__DirectorAction__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::comedyarch::autogen::DirectorAction::ice_staticId(), ::comedyarch::autogen::DirectorAction::ice_factory());
    }

    ~__F__comedyarch__autogen__DirectorAction__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::comedyarch::autogen::DirectorAction::ice_staticId());
    }
};

static __F__comedyarch__autogen__DirectorAction__Init __F__comedyarch__autogen__DirectorAction__i;

#ifdef __APPLE__
extern "C" { void __F__comedyarch__autogen__DirectorAction__initializer() {} }
#endif

void 
comedyarch::autogen::__patch__DirectorActionPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::comedyarch::autogen::DirectorActionPtr* p = static_cast< ::comedyarch::autogen::DirectorActionPtr*>(__addr);
    assert(p);
    *p = ::comedyarch::autogen::DirectorActionPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::comedyarch::autogen::DirectorAction::ice_staticId(), v->ice_id());
    }
}

bool
comedyarch::autogen::operator==(const ::comedyarch::autogen::DirectorAction& l, const ::comedyarch::autogen::DirectorAction& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
comedyarch::autogen::operator<(const ::comedyarch::autogen::DirectorAction& l, const ::comedyarch::autogen::DirectorAction& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

comedyarch::autogen::Reaction::Reaction(const ::std::string& __ice_react) :
    react(__ice_react)
{
}

::Ice::ObjectPtr
comedyarch::autogen::Reaction::ice_clone() const
{
    ::comedyarch::autogen::ReactionPtr __p = new ::comedyarch::autogen::Reaction(*this);
    return __p;
}

static const ::std::string __comedyarch__autogen__Reaction_ids[2] =
{
    "::Ice::Object",
    "::comedyarch::autogen::Reaction"
};

bool
comedyarch::autogen::Reaction::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__comedyarch__autogen__Reaction_ids, __comedyarch__autogen__Reaction_ids + 2, _s);
}

::std::vector< ::std::string>
comedyarch::autogen::Reaction::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__comedyarch__autogen__Reaction_ids[0], &__comedyarch__autogen__Reaction_ids[2]);
}

const ::std::string&
comedyarch::autogen::Reaction::ice_id(const ::Ice::Current&) const
{
    return __comedyarch__autogen__Reaction_ids[1];
}

const ::std::string&
comedyarch::autogen::Reaction::ice_staticId()
{
    return __comedyarch__autogen__Reaction_ids[1];
}

void
comedyarch::autogen::Reaction::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(react);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
comedyarch::autogen::Reaction::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(react);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
comedyarch::autogen::Reaction::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::Reaction was not generated with stream support";
    throw ex;
}

void
comedyarch::autogen::Reaction::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::Reaction was not generated with stream support";
    throw ex;
}

class __F__comedyarch__autogen__Reaction : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::comedyarch::autogen::Reaction::ice_staticId());
        return new ::comedyarch::autogen::Reaction;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__comedyarch__autogen__Reaction_Ptr = new __F__comedyarch__autogen__Reaction;

const ::Ice::ObjectFactoryPtr&
comedyarch::autogen::Reaction::ice_factory()
{
    return __F__comedyarch__autogen__Reaction_Ptr;
}

class __F__comedyarch__autogen__Reaction__Init
{
public:

    __F__comedyarch__autogen__Reaction__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::comedyarch::autogen::Reaction::ice_staticId(), ::comedyarch::autogen::Reaction::ice_factory());
    }

    ~__F__comedyarch__autogen__Reaction__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::comedyarch::autogen::Reaction::ice_staticId());
    }
};

static __F__comedyarch__autogen__Reaction__Init __F__comedyarch__autogen__Reaction__i;

#ifdef __APPLE__
extern "C" { void __F__comedyarch__autogen__Reaction__initializer() {} }
#endif

void 
comedyarch::autogen::__patch__ReactionPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::comedyarch::autogen::ReactionPtr* p = static_cast< ::comedyarch::autogen::ReactionPtr*>(__addr);
    assert(p);
    *p = ::comedyarch::autogen::ReactionPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::comedyarch::autogen::Reaction::ice_staticId(), v->ice_id());
    }
}

bool
comedyarch::autogen::operator==(const ::comedyarch::autogen::Reaction& l, const ::comedyarch::autogen::Reaction& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
comedyarch::autogen::operator<(const ::comedyarch::autogen::Reaction& l, const ::comedyarch::autogen::Reaction& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
comedyarch::autogen::TheGreatPretender::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __comedyarch__autogen__TheGreatPretender_ids[2] =
{
    "::Ice::Object",
    "::comedyarch::autogen::TheGreatPretender"
};

bool
comedyarch::autogen::TheGreatPretender::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__comedyarch__autogen__TheGreatPretender_ids, __comedyarch__autogen__TheGreatPretender_ids + 2, _s);
}

::std::vector< ::std::string>
comedyarch::autogen::TheGreatPretender::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__comedyarch__autogen__TheGreatPretender_ids[0], &__comedyarch__autogen__TheGreatPretender_ids[2]);
}

const ::std::string&
comedyarch::autogen::TheGreatPretender::ice_id(const ::Ice::Current&) const
{
    return __comedyarch__autogen__TheGreatPretender_ids[1];
}

const ::std::string&
comedyarch::autogen::TheGreatPretender::ice_staticId()
{
    return __comedyarch__autogen__TheGreatPretender_ids[1];
}

::Ice::DispatchStatus
comedyarch::autogen::TheGreatPretender::___getLies(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    getLies(__current);
    return ::Ice::DispatchOK;
}

static ::std::string __comedyarch__autogen__TheGreatPretender_all[] =
{
    "getLies",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

::Ice::DispatchStatus
comedyarch::autogen::TheGreatPretender::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__comedyarch__autogen__TheGreatPretender_all, __comedyarch__autogen__TheGreatPretender_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __comedyarch__autogen__TheGreatPretender_all)
    {
        case 0:
        {
            return ___getLies(in, current);
        }
        case 1:
        {
            return ___ice_id(in, current);
        }
        case 2:
        {
            return ___ice_ids(in, current);
        }
        case 3:
        {
            return ___ice_isA(in, current);
        }
        case 4:
        {
            return ___ice_ping(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
comedyarch::autogen::TheGreatPretender::__write(::IceInternal::BasicStream* __os) const
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
comedyarch::autogen::TheGreatPretender::__read(::IceInternal::BasicStream* __is, bool __rid)
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
comedyarch::autogen::TheGreatPretender::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::TheGreatPretender was not generated with stream support";
    throw ex;
}

void
comedyarch::autogen::TheGreatPretender::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type comedyarch::autogen::TheGreatPretender was not generated with stream support";
    throw ex;
}

void 
comedyarch::autogen::__patch__TheGreatPretenderPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::comedyarch::autogen::TheGreatPretenderPtr* p = static_cast< ::comedyarch::autogen::TheGreatPretenderPtr*>(__addr);
    assert(p);
    *p = ::comedyarch::autogen::TheGreatPretenderPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::comedyarch::autogen::TheGreatPretender::ice_staticId(), v->ice_id());
    }
}

bool
comedyarch::autogen::operator==(const ::comedyarch::autogen::TheGreatPretender& l, const ::comedyarch::autogen::TheGreatPretender& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
comedyarch::autogen::operator<(const ::comedyarch::autogen::TheGreatPretender& l, const ::comedyarch::autogen::TheGreatPretender& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
