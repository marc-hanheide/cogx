// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `PCogX.ice'

#ifndef ___data_private_grettonc_CogX_SVN_cogx_code_planning_trunk_subarchitectures_PCogX_src_c___0x_PCogX_hpp__
#define ___data_private_grettonc_CogX_SVN_cogx_code_planning_trunk_subarchitectures_PCogX_src_c___0x_PCogX_hpp__

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

namespace PCogX
{

class obtainPlanner;

class readPropositionIdentifiers;

class postSTRIPSAction;

class postActionDefinition;

class postTypes;

class postXXsubtypeofYY;

}

}

namespace PCogX
{

class obtainPlanner;
bool operator==(const obtainPlanner&, const obtainPlanner&);
bool operator<(const obtainPlanner&, const obtainPlanner&);

class readPropositionIdentifiers;
bool operator==(const readPropositionIdentifiers&, const readPropositionIdentifiers&);
bool operator<(const readPropositionIdentifiers&, const readPropositionIdentifiers&);

class postSTRIPSAction;
bool operator==(const postSTRIPSAction&, const postSTRIPSAction&);
bool operator<(const postSTRIPSAction&, const postSTRIPSAction&);

class postActionDefinition;
bool operator==(const postActionDefinition&, const postActionDefinition&);
bool operator<(const postActionDefinition&, const postActionDefinition&);

class postTypes;
bool operator==(const postTypes&, const postTypes&);
bool operator<(const postTypes&, const postTypes&);

class postXXsubtypeofYY;
bool operator==(const postXXsubtypeofYY&, const postXXsubtypeofYY&);
bool operator<(const postXXsubtypeofYY&, const postXXsubtypeofYY&);

}

namespace IceInternal
{

::Ice::Object* upCast(::PCogX::obtainPlanner*);
::IceProxy::Ice::Object* upCast(::IceProxy::PCogX::obtainPlanner*);

::Ice::Object* upCast(::PCogX::readPropositionIdentifiers*);
::IceProxy::Ice::Object* upCast(::IceProxy::PCogX::readPropositionIdentifiers*);

::Ice::Object* upCast(::PCogX::postSTRIPSAction*);
::IceProxy::Ice::Object* upCast(::IceProxy::PCogX::postSTRIPSAction*);

::Ice::Object* upCast(::PCogX::postActionDefinition*);
::IceProxy::Ice::Object* upCast(::IceProxy::PCogX::postActionDefinition*);

::Ice::Object* upCast(::PCogX::postTypes*);
::IceProxy::Ice::Object* upCast(::IceProxy::PCogX::postTypes*);

::Ice::Object* upCast(::PCogX::postXXsubtypeofYY*);
::IceProxy::Ice::Object* upCast(::IceProxy::PCogX::postXXsubtypeofYY*);

}

namespace PCogX
{

typedef ::IceInternal::Handle< ::PCogX::obtainPlanner> obtainPlannerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::PCogX::obtainPlanner> obtainPlannerPrx;

void __read(::IceInternal::BasicStream*, obtainPlannerPrx&);
void __patch__obtainPlannerPtr(void*, ::Ice::ObjectPtr&);

void ice_writeobtainPlannerPrx(const ::Ice::OutputStreamPtr&, const obtainPlannerPrx&);
void ice_readobtainPlannerPrx(const ::Ice::InputStreamPtr&, obtainPlannerPrx&);
void ice_writeobtainPlanner(const ::Ice::OutputStreamPtr&, const obtainPlannerPtr&);
void ice_readobtainPlanner(const ::Ice::InputStreamPtr&, obtainPlannerPtr&);

typedef ::IceInternal::Handle< ::PCogX::readPropositionIdentifiers> readPropositionIdentifiersPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::PCogX::readPropositionIdentifiers> readPropositionIdentifiersPrx;

void __read(::IceInternal::BasicStream*, readPropositionIdentifiersPrx&);
void __patch__readPropositionIdentifiersPtr(void*, ::Ice::ObjectPtr&);

void ice_writereadPropositionIdentifiersPrx(const ::Ice::OutputStreamPtr&, const readPropositionIdentifiersPrx&);
void ice_readreadPropositionIdentifiersPrx(const ::Ice::InputStreamPtr&, readPropositionIdentifiersPrx&);
void ice_writereadPropositionIdentifiers(const ::Ice::OutputStreamPtr&, const readPropositionIdentifiersPtr&);
void ice_readreadPropositionIdentifiers(const ::Ice::InputStreamPtr&, readPropositionIdentifiersPtr&);

typedef ::IceInternal::Handle< ::PCogX::postSTRIPSAction> postSTRIPSActionPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::PCogX::postSTRIPSAction> postSTRIPSActionPrx;

void __read(::IceInternal::BasicStream*, postSTRIPSActionPrx&);
void __patch__postSTRIPSActionPtr(void*, ::Ice::ObjectPtr&);

void ice_writepostSTRIPSActionPrx(const ::Ice::OutputStreamPtr&, const postSTRIPSActionPrx&);
void ice_readpostSTRIPSActionPrx(const ::Ice::InputStreamPtr&, postSTRIPSActionPrx&);
void ice_writepostSTRIPSAction(const ::Ice::OutputStreamPtr&, const postSTRIPSActionPtr&);
void ice_readpostSTRIPSAction(const ::Ice::InputStreamPtr&, postSTRIPSActionPtr&);

typedef ::IceInternal::Handle< ::PCogX::postActionDefinition> postActionDefinitionPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::PCogX::postActionDefinition> postActionDefinitionPrx;

void __read(::IceInternal::BasicStream*, postActionDefinitionPrx&);
void __patch__postActionDefinitionPtr(void*, ::Ice::ObjectPtr&);

void ice_writepostActionDefinitionPrx(const ::Ice::OutputStreamPtr&, const postActionDefinitionPrx&);
void ice_readpostActionDefinitionPrx(const ::Ice::InputStreamPtr&, postActionDefinitionPrx&);
void ice_writepostActionDefinition(const ::Ice::OutputStreamPtr&, const postActionDefinitionPtr&);
void ice_readpostActionDefinition(const ::Ice::InputStreamPtr&, postActionDefinitionPtr&);

typedef ::IceInternal::Handle< ::PCogX::postTypes> postTypesPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::PCogX::postTypes> postTypesPrx;

void __read(::IceInternal::BasicStream*, postTypesPrx&);
void __patch__postTypesPtr(void*, ::Ice::ObjectPtr&);

void ice_writepostTypesPrx(const ::Ice::OutputStreamPtr&, const postTypesPrx&);
void ice_readpostTypesPrx(const ::Ice::InputStreamPtr&, postTypesPrx&);
void ice_writepostTypes(const ::Ice::OutputStreamPtr&, const postTypesPtr&);
void ice_readpostTypes(const ::Ice::InputStreamPtr&, postTypesPtr&);

typedef ::IceInternal::Handle< ::PCogX::postXXsubtypeofYY> postXXsubtypeofYYPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::PCogX::postXXsubtypeofYY> postXXsubtypeofYYPrx;

void __read(::IceInternal::BasicStream*, postXXsubtypeofYYPrx&);
void __patch__postXXsubtypeofYYPtr(void*, ::Ice::ObjectPtr&);

void ice_writepostXXsubtypeofYYPrx(const ::Ice::OutputStreamPtr&, const postXXsubtypeofYYPrx&);
void ice_readpostXXsubtypeofYYPrx(const ::Ice::InputStreamPtr&, postXXsubtypeofYYPrx&);
void ice_writepostXXsubtypeofYY(const ::Ice::OutputStreamPtr&, const postXXsubtypeofYYPtr&);
void ice_readpostXXsubtypeofYY(const ::Ice::InputStreamPtr&, postXXsubtypeofYYPtr&);

}

namespace PCogX
{

typedef ::std::vector< ::std::string> OptionalMemberDesignator;

typedef ::std::vector< ::std::string> OptionalMemberDomainDescriptor;

typedef ::std::vector< ::std::string> OptionalMemberPlannerDescriptor;

typedef ::std::map< ::Ice::Int, ::std::string> PropositionIDs;
void __writePropositionIDs(::IceInternal::BasicStream*, const PropositionIDs&);
void __readPropositionIDs(::IceInternal::BasicStream*, PropositionIDs&);
void ice_writePropositionIDs(const ::Ice::OutputStreamPtr&, const PropositionIDs&);
void ice_readPropositionIDs(const ::Ice::InputStreamPtr&, PropositionIDs&);

typedef ::std::vector< ::std::string> AddList;

typedef ::std::vector< ::std::string> DeleteList;

typedef ::std::vector< ::std::string> Precondition;

typedef ::std::vector< ::std::string> XX;

typedef ::std::vector< ::std::string> YY;

}

namespace IceProxy
{

namespace PCogX
{

class obtainPlanner : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<obtainPlanner> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<obtainPlanner*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<obtainPlanner*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class readPropositionIdentifiers : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<readPropositionIdentifiers> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<readPropositionIdentifiers*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<readPropositionIdentifiers*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class postSTRIPSAction : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postSTRIPSAction> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postSTRIPSAction*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<postSTRIPSAction*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class postActionDefinition : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postActionDefinition> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postActionDefinition*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<postActionDefinition*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class postTypes : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<postTypes> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postTypes> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postTypes*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<postTypes*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class postXXsubtypeofYY : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<postXXsubtypeofYY> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<postXXsubtypeofYY*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<postXXsubtypeofYY*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace PCogX
{

class obtainPlanner : virtual public ::IceDelegate::Ice::Object
{
public:
};

class readPropositionIdentifiers : virtual public ::IceDelegate::Ice::Object
{
public:
};

class postSTRIPSAction : virtual public ::IceDelegate::Ice::Object
{
public:
};

class postActionDefinition : virtual public ::IceDelegate::Ice::Object
{
public:
};

class postTypes : virtual public ::IceDelegate::Ice::Object
{
public:
};

class postXXsubtypeofYY : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

}

namespace IceDelegateM
{

namespace PCogX
{

class obtainPlanner : virtual public ::IceDelegate::PCogX::obtainPlanner,
                      virtual public ::IceDelegateM::Ice::Object
{
public:
};

class readPropositionIdentifiers : virtual public ::IceDelegate::PCogX::readPropositionIdentifiers,
                                   virtual public ::IceDelegateM::Ice::Object
{
public:
};

class postSTRIPSAction : virtual public ::IceDelegate::PCogX::postSTRIPSAction,
                         virtual public ::IceDelegateM::Ice::Object
{
public:
};

class postActionDefinition : virtual public ::IceDelegate::PCogX::postActionDefinition,
                             virtual public ::IceDelegateM::Ice::Object
{
public:
};

class postTypes : virtual public ::IceDelegate::PCogX::postTypes,
                  virtual public ::IceDelegateM::Ice::Object
{
public:
};

class postXXsubtypeofYY : virtual public ::IceDelegate::PCogX::postXXsubtypeofYY,
                          virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

}

namespace IceDelegateD
{

namespace PCogX
{

class obtainPlanner : virtual public ::IceDelegate::PCogX::obtainPlanner,
                      virtual public ::IceDelegateD::Ice::Object
{
public:
};

class readPropositionIdentifiers : virtual public ::IceDelegate::PCogX::readPropositionIdentifiers,
                                   virtual public ::IceDelegateD::Ice::Object
{
public:
};

class postSTRIPSAction : virtual public ::IceDelegate::PCogX::postSTRIPSAction,
                         virtual public ::IceDelegateD::Ice::Object
{
public:
};

class postActionDefinition : virtual public ::IceDelegate::PCogX::postActionDefinition,
                             virtual public ::IceDelegateD::Ice::Object
{
public:
};

class postTypes : virtual public ::IceDelegate::PCogX::postTypes,
                  virtual public ::IceDelegateD::Ice::Object
{
public:
};

class postXXsubtypeofYY : virtual public ::IceDelegate::PCogX::postXXsubtypeofYY,
                          virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

}

namespace PCogX
{

class obtainPlanner : virtual public ::Ice::Object
{
public:

    typedef obtainPlannerPrx ProxyType;
    typedef obtainPlannerPtr PointerType;
    
    obtainPlanner() {}
    obtainPlanner(const ::std::string&, const ::PCogX::OptionalMemberDesignator&, const ::PCogX::OptionalMemberPlannerDescriptor&);
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

    virtual ~obtainPlanner() {}

    friend class obtainPlanner__staticInit;

public:

    ::std::string identityOfCreatedPlannerIsAReturn;

    ::PCogX::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;

    ::PCogX::OptionalMemberPlannerDescriptor optionalMemberPlannerDescriptorIsAnArgument;
};

class obtainPlanner__staticInit
{
public:

    ::PCogX::obtainPlanner _init;
};

static obtainPlanner__staticInit _obtainPlanner_init;

class readPropositionIdentifiers : virtual public ::Ice::Object
{
public:

    typedef readPropositionIdentifiersPrx ProxyType;
    typedef readPropositionIdentifiersPtr PointerType;
    
    readPropositionIdentifiers() {}
    readPropositionIdentifiers(const ::PCogX::PropositionIDs&, const ::PCogX::OptionalMemberDesignator&, const ::PCogX::OptionalMemberDomainDescriptor&);
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

    virtual ~readPropositionIdentifiers() {}

public:

    ::PCogX::PropositionIDs propositionIDsIsAReturn;

    ::PCogX::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;

    ::PCogX::OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
};

class postSTRIPSAction : virtual public ::Ice::Object
{
public:

    typedef postSTRIPSActionPrx ProxyType;
    typedef postSTRIPSActionPtr PointerType;
    
    postSTRIPSAction() {}
    postSTRIPSAction(const ::PCogX::AddList&, const ::PCogX::DeleteList&, const ::PCogX::Precondition&, const ::PCogX::OptionalMemberDesignator&, const ::PCogX::OptionalMemberDomainDescriptor&);
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

    virtual ~postSTRIPSAction() {}

public:

    ::PCogX::AddList addListIsAnArgument;

    ::PCogX::DeleteList deleteListIsAnArgument;

    ::PCogX::Precondition preconditionIsAnArgument;

    ::PCogX::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;

    ::PCogX::OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
};

class postActionDefinition : virtual public ::Ice::Object
{
public:

    typedef postActionDefinitionPrx ProxyType;
    typedef postActionDefinitionPtr PointerType;
    
    postActionDefinition() {}
    postActionDefinition(const ::std::string&, bool, const ::std::string&, const ::PCogX::OptionalMemberDesignator&, const ::PCogX::OptionalMemberDomainDescriptor&);
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

    virtual ~postActionDefinition() {}

public:

    ::std::string actionDefinitionIsAnArgument;

    bool successIsAReturn;

    ::std::string parserComplaintIsAReturn;

    ::PCogX::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;

    ::PCogX::OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
};

class postTypes : virtual public ::Ice::Object
{
public:

    typedef postTypesPrx ProxyType;
    typedef postTypesPtr PointerType;
    
    postTypes() {}
    postTypes(const ::std::string&, bool, const ::std::string&, const ::PCogX::OptionalMemberDesignator&, const ::PCogX::OptionalMemberDomainDescriptor&);
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

    virtual ~postTypes() {}

public:

    ::std::string typesDefinitionIsAnArgument;

    bool successIsAReturn;

    ::std::string ParserComplaintIsAReturn;

    ::PCogX::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;

    ::PCogX::OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
};

class postXXsubtypeofYY : virtual public ::Ice::Object
{
public:

    typedef postXXsubtypeofYYPrx ProxyType;
    typedef postXXsubtypeofYYPtr PointerType;
    
    postXXsubtypeofYY() {}
    postXXsubtypeofYY(const ::PCogX::XX&, const ::PCogX::YY&, const ::PCogX::OptionalMemberDesignator&, const ::PCogX::OptionalMemberDomainDescriptor&);
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

    virtual ~postXXsubtypeofYY() {}

public:

    ::PCogX::XX XXIsAnArgument;

    ::PCogX::YY YYIsAnArgument;

    ::PCogX::OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;

    ::PCogX::OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
};

}

#endif
