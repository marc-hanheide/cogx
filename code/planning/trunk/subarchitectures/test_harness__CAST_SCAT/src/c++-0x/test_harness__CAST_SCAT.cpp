// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `test_harness__CAST_SCAT.ice'

#include <test_harness__CAST_SCAT.hpp>
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

::Ice::Object* IceInternal::upCast(::testHarnessCASTSCAT::computeFibonacci* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::testHarnessCASTSCAT::computeFibonacci* p) { return p; }

::Ice::Object* IceInternal::upCast(::testHarnessCASTSCAT::helloWorld* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::testHarnessCASTSCAT::helloWorld* p) { return p; }

void
testHarnessCASTSCAT::__read(::IceInternal::BasicStream* __is, ::testHarnessCASTSCAT::computeFibonacciPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::testHarnessCASTSCAT::computeFibonacci;
        v->__copyFrom(proxy);
    }
}

void
testHarnessCASTSCAT::ice_writecomputeFibonacciPrx(const ::Ice::OutputStreamPtr& __outS, const ::testHarnessCASTSCAT::computeFibonacciPrx& v)
{
    __outS->writeProxy(v);
}

void
testHarnessCASTSCAT::ice_readcomputeFibonacciPrx(const ::Ice::InputStreamPtr& __inS, ::testHarnessCASTSCAT::computeFibonacciPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::testHarnessCASTSCAT::computeFibonacci;
        v->__copyFrom(proxy);
    }
}

void
testHarnessCASTSCAT::ice_writecomputeFibonacci(const ::Ice::OutputStreamPtr& __outS, const ::testHarnessCASTSCAT::computeFibonacciPtr& v)
{
    __outS->writeObject(v);
}

void
testHarnessCASTSCAT::ice_readcomputeFibonacci(const ::Ice::InputStreamPtr& __inS, ::testHarnessCASTSCAT::computeFibonacciPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::testHarnessCASTSCAT::__patch__computeFibonacciPtr, &__v);
    __inS->readObject(__cb);
}

void
testHarnessCASTSCAT::__read(::IceInternal::BasicStream* __is, ::testHarnessCASTSCAT::helloWorldPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::testHarnessCASTSCAT::helloWorld;
        v->__copyFrom(proxy);
    }
}

void
testHarnessCASTSCAT::ice_writehelloWorldPrx(const ::Ice::OutputStreamPtr& __outS, const ::testHarnessCASTSCAT::helloWorldPrx& v)
{
    __outS->writeProxy(v);
}

void
testHarnessCASTSCAT::ice_readhelloWorldPrx(const ::Ice::InputStreamPtr& __inS, ::testHarnessCASTSCAT::helloWorldPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::testHarnessCASTSCAT::helloWorld;
        v->__copyFrom(proxy);
    }
}

void
testHarnessCASTSCAT::ice_writehelloWorld(const ::Ice::OutputStreamPtr& __outS, const ::testHarnessCASTSCAT::helloWorldPtr& v)
{
    __outS->writeObject(v);
}

void
testHarnessCASTSCAT::ice_readhelloWorld(const ::Ice::InputStreamPtr& __inS, ::testHarnessCASTSCAT::helloWorldPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::testHarnessCASTSCAT::__patch__helloWorldPtr, &__v);
    __inS->readObject(__cb);
}

const ::std::string&
IceProxy::testHarnessCASTSCAT::computeFibonacci::ice_staticId()
{
    return ::testHarnessCASTSCAT::computeFibonacci::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::testHarnessCASTSCAT::computeFibonacci::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::testHarnessCASTSCAT::computeFibonacci);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::testHarnessCASTSCAT::computeFibonacci::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::testHarnessCASTSCAT::computeFibonacci);
}

::IceProxy::Ice::Object*
IceProxy::testHarnessCASTSCAT::computeFibonacci::__newInstance() const
{
    return new computeFibonacci;
}

const ::std::string&
IceProxy::testHarnessCASTSCAT::helloWorld::ice_staticId()
{
    return ::testHarnessCASTSCAT::helloWorld::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::testHarnessCASTSCAT::helloWorld::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::testHarnessCASTSCAT::helloWorld);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::testHarnessCASTSCAT::helloWorld::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::testHarnessCASTSCAT::helloWorld);
}

::IceProxy::Ice::Object*
IceProxy::testHarnessCASTSCAT::helloWorld::__newInstance() const
{
    return new helloWorld;
}

testHarnessCASTSCAT::computeFibonacci::computeFibonacci(::Ice::Int __ice_ithNumberIsAnArgument, ::Ice::Int __ice_answerIsAReturn, const ::testHarnessCASTSCAT::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument) :
    ithNumberIsAnArgument(__ice_ithNumberIsAnArgument),
    answerIsAReturn(__ice_answerIsAReturn),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument)
{
}

::Ice::ObjectPtr
testHarnessCASTSCAT::computeFibonacci::ice_clone() const
{
    ::testHarnessCASTSCAT::computeFibonacciPtr __p = new ::testHarnessCASTSCAT::computeFibonacci(*this);
    return __p;
}

static const ::std::string __testHarnessCASTSCAT__computeFibonacci_ids[2] =
{
    "::Ice::Object",
    "::testHarnessCASTSCAT::computeFibonacci"
};

bool
testHarnessCASTSCAT::computeFibonacci::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__testHarnessCASTSCAT__computeFibonacci_ids, __testHarnessCASTSCAT__computeFibonacci_ids + 2, _s);
}

::std::vector< ::std::string>
testHarnessCASTSCAT::computeFibonacci::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__testHarnessCASTSCAT__computeFibonacci_ids[0], &__testHarnessCASTSCAT__computeFibonacci_ids[2]);
}

const ::std::string&
testHarnessCASTSCAT::computeFibonacci::ice_id(const ::Ice::Current&) const
{
    return __testHarnessCASTSCAT__computeFibonacci_ids[1];
}

const ::std::string&
testHarnessCASTSCAT::computeFibonacci::ice_staticId()
{
    return __testHarnessCASTSCAT__computeFibonacci_ids[1];
}

void
testHarnessCASTSCAT::computeFibonacci::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(ithNumberIsAnArgument);
    __os->write(answerIsAReturn);
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
testHarnessCASTSCAT::computeFibonacci::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(ithNumberIsAnArgument);
    __is->read(answerIsAReturn);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
testHarnessCASTSCAT::computeFibonacci::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeInt(ithNumberIsAnArgument);
    __outS->writeInt(answerIsAReturn);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
testHarnessCASTSCAT::computeFibonacci::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    ithNumberIsAnArgument = __inS->readInt();
    answerIsAReturn = __inS->readInt();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__testHarnessCASTSCAT__computeFibonacci : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::testHarnessCASTSCAT::computeFibonacci::ice_staticId());
        return new ::testHarnessCASTSCAT::computeFibonacci;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__testHarnessCASTSCAT__computeFibonacci_Ptr = new __F__testHarnessCASTSCAT__computeFibonacci;

const ::Ice::ObjectFactoryPtr&
testHarnessCASTSCAT::computeFibonacci::ice_factory()
{
    return __F__testHarnessCASTSCAT__computeFibonacci_Ptr;
}

class __F__testHarnessCASTSCAT__computeFibonacci__Init
{
public:

    __F__testHarnessCASTSCAT__computeFibonacci__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::testHarnessCASTSCAT::computeFibonacci::ice_staticId(), ::testHarnessCASTSCAT::computeFibonacci::ice_factory());
    }

    ~__F__testHarnessCASTSCAT__computeFibonacci__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::testHarnessCASTSCAT::computeFibonacci::ice_staticId());
    }
};

static __F__testHarnessCASTSCAT__computeFibonacci__Init __F__testHarnessCASTSCAT__computeFibonacci__i;

#ifdef __APPLE__
extern "C" { void __F__testHarnessCASTSCAT__computeFibonacci__initializer() {} }
#endif

void 
testHarnessCASTSCAT::__patch__computeFibonacciPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::testHarnessCASTSCAT::computeFibonacciPtr* p = static_cast< ::testHarnessCASTSCAT::computeFibonacciPtr*>(__addr);
    assert(p);
    *p = ::testHarnessCASTSCAT::computeFibonacciPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::testHarnessCASTSCAT::computeFibonacci::ice_staticId(), v->ice_id());
    }
}

bool
testHarnessCASTSCAT::operator==(const ::testHarnessCASTSCAT::computeFibonacci& l, const ::testHarnessCASTSCAT::computeFibonacci& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
testHarnessCASTSCAT::operator<(const ::testHarnessCASTSCAT::computeFibonacci& l, const ::testHarnessCASTSCAT::computeFibonacci& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

testHarnessCASTSCAT::helloWorld::helloWorld(const ::std::string& __ice_thingToPrintIsAnArgument, const ::testHarnessCASTSCAT::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument) :
    thingToPrintIsAnArgument(__ice_thingToPrintIsAnArgument),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument)
{
}

::Ice::ObjectPtr
testHarnessCASTSCAT::helloWorld::ice_clone() const
{
    ::testHarnessCASTSCAT::helloWorldPtr __p = new ::testHarnessCASTSCAT::helloWorld(*this);
    return __p;
}

static const ::std::string __testHarnessCASTSCAT__helloWorld_ids[2] =
{
    "::Ice::Object",
    "::testHarnessCASTSCAT::helloWorld"
};

bool
testHarnessCASTSCAT::helloWorld::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__testHarnessCASTSCAT__helloWorld_ids, __testHarnessCASTSCAT__helloWorld_ids + 2, _s);
}

::std::vector< ::std::string>
testHarnessCASTSCAT::helloWorld::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__testHarnessCASTSCAT__helloWorld_ids[0], &__testHarnessCASTSCAT__helloWorld_ids[2]);
}

const ::std::string&
testHarnessCASTSCAT::helloWorld::ice_id(const ::Ice::Current&) const
{
    return __testHarnessCASTSCAT__helloWorld_ids[1];
}

const ::std::string&
testHarnessCASTSCAT::helloWorld::ice_staticId()
{
    return __testHarnessCASTSCAT__helloWorld_ids[1];
}

void
testHarnessCASTSCAT::helloWorld::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(thingToPrintIsAnArgument);
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
testHarnessCASTSCAT::helloWorld::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(thingToPrintIsAnArgument);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
testHarnessCASTSCAT::helloWorld::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(thingToPrintIsAnArgument);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
testHarnessCASTSCAT::helloWorld::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    thingToPrintIsAnArgument = __inS->readString();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__testHarnessCASTSCAT__helloWorld : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::testHarnessCASTSCAT::helloWorld::ice_staticId());
        return new ::testHarnessCASTSCAT::helloWorld;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__testHarnessCASTSCAT__helloWorld_Ptr = new __F__testHarnessCASTSCAT__helloWorld;

const ::Ice::ObjectFactoryPtr&
testHarnessCASTSCAT::helloWorld::ice_factory()
{
    return __F__testHarnessCASTSCAT__helloWorld_Ptr;
}

class __F__testHarnessCASTSCAT__helloWorld__Init
{
public:

    __F__testHarnessCASTSCAT__helloWorld__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::testHarnessCASTSCAT::helloWorld::ice_staticId(), ::testHarnessCASTSCAT::helloWorld::ice_factory());
    }

    ~__F__testHarnessCASTSCAT__helloWorld__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::testHarnessCASTSCAT::helloWorld::ice_staticId());
    }
};

static __F__testHarnessCASTSCAT__helloWorld__Init __F__testHarnessCASTSCAT__helloWorld__i;

#ifdef __APPLE__
extern "C" { void __F__testHarnessCASTSCAT__helloWorld__initializer() {} }
#endif

void 
testHarnessCASTSCAT::__patch__helloWorldPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::testHarnessCASTSCAT::helloWorldPtr* p = static_cast< ::testHarnessCASTSCAT::helloWorldPtr*>(__addr);
    assert(p);
    *p = ::testHarnessCASTSCAT::helloWorldPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::testHarnessCASTSCAT::helloWorld::ice_staticId(), v->ice_id());
    }
}

bool
testHarnessCASTSCAT::operator==(const ::testHarnessCASTSCAT::helloWorld& l, const ::testHarnessCASTSCAT::helloWorld& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
testHarnessCASTSCAT::operator<(const ::testHarnessCASTSCAT::helloWorld& l, const ::testHarnessCASTSCAT::helloWorld& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
