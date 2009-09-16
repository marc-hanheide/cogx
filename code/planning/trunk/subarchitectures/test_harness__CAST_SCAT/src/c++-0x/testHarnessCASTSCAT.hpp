// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `testHarnessCASTSCAT.ice'

#ifndef ___data_private_grettonc_CogX_SVN_cogx_code_planning_trunk_subarchitectures_test_harness__CAST_SCAT_src_c___0x_testHarnessCASTSCAT_hpp__
#define ___data_private_grettonc_CogX_SVN_cogx_code_planning_trunk_subarchitectures_test_harness__CAST_SCAT_src_c___0x_testHarnessCASTSCAT_hpp__

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

namespace testHarnessCASTSCAT
{

class computeFibonacci;

class helloWorld;

}

}

namespace testHarnessCASTSCAT
{

class computeFibonacci;
bool operator==(const computeFibonacci&, const computeFibonacci&);
bool operator<(const computeFibonacci&, const computeFibonacci&);

class helloWorld;
bool operator==(const helloWorld&, const helloWorld&);
bool operator<(const helloWorld&, const helloWorld&);

}

namespace IceInternal
{

::Ice::Object* upCast(::testHarnessCASTSCAT::computeFibonacci*);
::IceProxy::Ice::Object* upCast(::IceProxy::testHarnessCASTSCAT::computeFibonacci*);

::Ice::Object* upCast(::testHarnessCASTSCAT::helloWorld*);
::IceProxy::Ice::Object* upCast(::IceProxy::testHarnessCASTSCAT::helloWorld*);

}

namespace testHarnessCASTSCAT
{

typedef ::IceInternal::Handle< ::testHarnessCASTSCAT::computeFibonacci> computeFibonacciPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::testHarnessCASTSCAT::computeFibonacci> computeFibonacciPrx;

void __read(::IceInternal::BasicStream*, computeFibonacciPrx&);
void __patch__computeFibonacciPtr(void*, ::Ice::ObjectPtr&);

void ice_writecomputeFibonacciPrx(const ::Ice::OutputStreamPtr&, const computeFibonacciPrx&);
void ice_readcomputeFibonacciPrx(const ::Ice::InputStreamPtr&, computeFibonacciPrx&);
void ice_writecomputeFibonacci(const ::Ice::OutputStreamPtr&, const computeFibonacciPtr&);
void ice_readcomputeFibonacci(const ::Ice::InputStreamPtr&, computeFibonacciPtr&);

typedef ::IceInternal::Handle< ::testHarnessCASTSCAT::helloWorld> helloWorldPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::testHarnessCASTSCAT::helloWorld> helloWorldPrx;

void __read(::IceInternal::BasicStream*, helloWorldPrx&);
void __patch__helloWorldPtr(void*, ::Ice::ObjectPtr&);

void ice_writehelloWorldPrx(const ::Ice::OutputStreamPtr&, const helloWorldPrx&);
void ice_readhelloWorldPrx(const ::Ice::InputStreamPtr&, helloWorldPrx&);
void ice_writehelloWorld(const ::Ice::OutputStreamPtr&, const helloWorldPtr&);
void ice_readhelloWorld(const ::Ice::InputStreamPtr&, helloWorldPtr&);

}

namespace testHarnessCASTSCAT
{

typedef ::std::vector< ::std::string> OptionalMemberDesignator;

}

namespace IceProxy
{

namespace testHarnessCASTSCAT
{

class computeFibonacci : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<computeFibonacci> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<computeFibonacci*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<computeFibonacci*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class helloWorld : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<helloWorld> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<helloWorld> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<helloWorld*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<helloWorld*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace testHarnessCASTSCAT
{

class computeFibonacci : virtual public ::IceDelegate::Ice::Object
{
public:
};

class helloWorld : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

}

namespace IceDelegateM
{

namespace testHarnessCASTSCAT
{

class computeFibonacci : virtual public ::IceDelegate::testHarnessCASTSCAT::computeFibonacci,
                         virtual public ::IceDelegateM::Ice::Object
{
public:
};

class helloWorld : virtual public ::IceDelegate::testHarnessCASTSCAT::helloWorld,
                   virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

}

namespace IceDelegateD
{

namespace testHarnessCASTSCAT
{

class computeFibonacci : virtual public ::IceDelegate::testHarnessCASTSCAT::computeFibonacci,
                         virtual public ::IceDelegateD::Ice::Object
{
public:
};

class helloWorld : virtual public ::IceDelegate::testHarnessCASTSCAT::helloWorld,
                   virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

}

namespace testHarnessCASTSCAT
{

class computeFibonacci : virtual public ::Ice::Object
{
public:

    typedef computeFibonacciPrx ProxyType;
    typedef computeFibonacciPtr PointerType;
    
    computeFibonacci() {}
    computeFibonacci(::Ice::Int, ::Ice::Int, const ::testHarnessCASTSCAT::OptionalMemberDesignator&);
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

    virtual ~computeFibonacci() {}

    friend class computeFibonacci__staticInit;

public:

    ::Ice::Int ithNumberIsAnArgument;

    ::Ice::Int answerIsAReturn;

    ::testHarnessCASTSCAT::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
};

class computeFibonacci__staticInit
{
public:

    ::testHarnessCASTSCAT::computeFibonacci _init;
};

static computeFibonacci__staticInit _computeFibonacci_init;

class helloWorld : virtual public ::Ice::Object
{
public:

    typedef helloWorldPrx ProxyType;
    typedef helloWorldPtr PointerType;
    
    helloWorld() {}
    helloWorld(const ::std::string&, const ::testHarnessCASTSCAT::OptionalMemberDesignator&);
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

    virtual ~helloWorld() {}

public:

    ::std::string thingToPrintIsAnArgument;

    ::testHarnessCASTSCAT::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
};

}

#endif
