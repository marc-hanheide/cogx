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

#include <PCogX.hpp>
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

::Ice::Object* IceInternal::upCast(::PCogX::obtainPlanner* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::PCogX::obtainPlanner* p) { return p; }

::Ice::Object* IceInternal::upCast(::PCogX::readPropositionIdentifiers* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::PCogX::readPropositionIdentifiers* p) { return p; }

::Ice::Object* IceInternal::upCast(::PCogX::postSTRIPSAction* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::PCogX::postSTRIPSAction* p) { return p; }

::Ice::Object* IceInternal::upCast(::PCogX::postActionDefinition* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::PCogX::postActionDefinition* p) { return p; }

::Ice::Object* IceInternal::upCast(::PCogX::postTypes* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::PCogX::postTypes* p) { return p; }

::Ice::Object* IceInternal::upCast(::PCogX::postXXsubtypeofYY* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::PCogX::postXXsubtypeofYY* p) { return p; }

void
PCogX::__read(::IceInternal::BasicStream* __is, ::PCogX::obtainPlannerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::obtainPlanner;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writeobtainPlannerPrx(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::obtainPlannerPrx& v)
{
    __outS->writeProxy(v);
}

void
PCogX::ice_readobtainPlannerPrx(const ::Ice::InputStreamPtr& __inS, ::PCogX::obtainPlannerPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::obtainPlanner;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writeobtainPlanner(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::obtainPlannerPtr& v)
{
    __outS->writeObject(v);
}

void
PCogX::ice_readobtainPlanner(const ::Ice::InputStreamPtr& __inS, ::PCogX::obtainPlannerPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::PCogX::__patch__obtainPlannerPtr, &__v);
    __inS->readObject(__cb);
}

void
PCogX::__read(::IceInternal::BasicStream* __is, ::PCogX::readPropositionIdentifiersPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::readPropositionIdentifiers;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writereadPropositionIdentifiersPrx(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::readPropositionIdentifiersPrx& v)
{
    __outS->writeProxy(v);
}

void
PCogX::ice_readreadPropositionIdentifiersPrx(const ::Ice::InputStreamPtr& __inS, ::PCogX::readPropositionIdentifiersPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::readPropositionIdentifiers;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writereadPropositionIdentifiers(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::readPropositionIdentifiersPtr& v)
{
    __outS->writeObject(v);
}

void
PCogX::ice_readreadPropositionIdentifiers(const ::Ice::InputStreamPtr& __inS, ::PCogX::readPropositionIdentifiersPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::PCogX::__patch__readPropositionIdentifiersPtr, &__v);
    __inS->readObject(__cb);
}

void
PCogX::__read(::IceInternal::BasicStream* __is, ::PCogX::postSTRIPSActionPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postSTRIPSAction;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostSTRIPSActionPrx(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postSTRIPSActionPrx& v)
{
    __outS->writeProxy(v);
}

void
PCogX::ice_readpostSTRIPSActionPrx(const ::Ice::InputStreamPtr& __inS, ::PCogX::postSTRIPSActionPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postSTRIPSAction;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostSTRIPSAction(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postSTRIPSActionPtr& v)
{
    __outS->writeObject(v);
}

void
PCogX::ice_readpostSTRIPSAction(const ::Ice::InputStreamPtr& __inS, ::PCogX::postSTRIPSActionPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::PCogX::__patch__postSTRIPSActionPtr, &__v);
    __inS->readObject(__cb);
}

void
PCogX::__read(::IceInternal::BasicStream* __is, ::PCogX::postActionDefinitionPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postActionDefinition;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostActionDefinitionPrx(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postActionDefinitionPrx& v)
{
    __outS->writeProxy(v);
}

void
PCogX::ice_readpostActionDefinitionPrx(const ::Ice::InputStreamPtr& __inS, ::PCogX::postActionDefinitionPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postActionDefinition;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostActionDefinition(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postActionDefinitionPtr& v)
{
    __outS->writeObject(v);
}

void
PCogX::ice_readpostActionDefinition(const ::Ice::InputStreamPtr& __inS, ::PCogX::postActionDefinitionPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::PCogX::__patch__postActionDefinitionPtr, &__v);
    __inS->readObject(__cb);
}

void
PCogX::__read(::IceInternal::BasicStream* __is, ::PCogX::postTypesPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postTypes;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostTypesPrx(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postTypesPrx& v)
{
    __outS->writeProxy(v);
}

void
PCogX::ice_readpostTypesPrx(const ::Ice::InputStreamPtr& __inS, ::PCogX::postTypesPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postTypes;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostTypes(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postTypesPtr& v)
{
    __outS->writeObject(v);
}

void
PCogX::ice_readpostTypes(const ::Ice::InputStreamPtr& __inS, ::PCogX::postTypesPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::PCogX::__patch__postTypesPtr, &__v);
    __inS->readObject(__cb);
}

void
PCogX::__read(::IceInternal::BasicStream* __is, ::PCogX::postXXsubtypeofYYPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postXXsubtypeofYY;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostXXsubtypeofYYPrx(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postXXsubtypeofYYPrx& v)
{
    __outS->writeProxy(v);
}

void
PCogX::ice_readpostXXsubtypeofYYPrx(const ::Ice::InputStreamPtr& __inS, ::PCogX::postXXsubtypeofYYPrx& v)
{
    ::Ice::ObjectPrx proxy = __inS->readProxy();
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::PCogX::postXXsubtypeofYY;
        v->__copyFrom(proxy);
    }
}

void
PCogX::ice_writepostXXsubtypeofYY(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::postXXsubtypeofYYPtr& v)
{
    __outS->writeObject(v);
}

void
PCogX::ice_readpostXXsubtypeofYY(const ::Ice::InputStreamPtr& __inS, ::PCogX::postXXsubtypeofYYPtr& __v)
{
    ::Ice::ReadObjectCallbackPtr __cb = new ::Ice::ReadObjectCallbackI(::PCogX::__patch__postXXsubtypeofYYPtr, &__v);
    __inS->readObject(__cb);
}

void
PCogX::__writePropositionIDs(::IceInternal::BasicStream* __os, const ::PCogX::PropositionIDs& v)
{
    __os->writeSize(::Ice::Int(v.size()));
    ::PCogX::PropositionIDs::const_iterator p;
    for(p = v.begin(); p != v.end(); ++p)
    {
        __os->write(p->first);
        __os->write(p->second);
    }
}

void
PCogX::__readPropositionIDs(::IceInternal::BasicStream* __is, ::PCogX::PropositionIDs& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    while(sz--)
    {
        ::std::pair<const  ::Ice::Int, ::std::string> pair;
        __is->read(const_cast< ::Ice::Int&>(pair.first));
        ::PCogX::PropositionIDs::iterator __i = v.insert(v.end(), pair);
        __is->read(__i->second);
    }
}

void
PCogX::ice_writePropositionIDs(const ::Ice::OutputStreamPtr& __outS, const ::PCogX::PropositionIDs& v)
{
    __outS->writeSize(::Ice::Int(v.size()));
    ::PCogX::PropositionIDs::const_iterator p;
    for(p = v.begin(); p != v.end(); ++p)
    {
        __outS->writeInt(p->first);
        __outS->writeString(p->second);
    }
}

void
PCogX::ice_readPropositionIDs(const ::Ice::InputStreamPtr& __inS, ::PCogX::PropositionIDs& v)
{
    ::Ice::Int sz = __inS->readSize();
    while(sz--)
    {
        ::std::pair<const  ::Ice::Int, ::std::string> pair;
        const_cast< ::Ice::Int&>(pair.first) = __inS->readInt();
        ::PCogX::PropositionIDs::iterator __i = v.insert(v.end(), pair);
        __i->second = __inS->readString();
    }
}

const ::std::string&
IceProxy::PCogX::obtainPlanner::ice_staticId()
{
    return ::PCogX::obtainPlanner::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::PCogX::obtainPlanner::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::PCogX::obtainPlanner);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::PCogX::obtainPlanner::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::PCogX::obtainPlanner);
}

::IceProxy::Ice::Object*
IceProxy::PCogX::obtainPlanner::__newInstance() const
{
    return new obtainPlanner;
}

const ::std::string&
IceProxy::PCogX::readPropositionIdentifiers::ice_staticId()
{
    return ::PCogX::readPropositionIdentifiers::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::PCogX::readPropositionIdentifiers::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::PCogX::readPropositionIdentifiers);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::PCogX::readPropositionIdentifiers::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::PCogX::readPropositionIdentifiers);
}

::IceProxy::Ice::Object*
IceProxy::PCogX::readPropositionIdentifiers::__newInstance() const
{
    return new readPropositionIdentifiers;
}

const ::std::string&
IceProxy::PCogX::postSTRIPSAction::ice_staticId()
{
    return ::PCogX::postSTRIPSAction::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::PCogX::postSTRIPSAction::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::PCogX::postSTRIPSAction);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::PCogX::postSTRIPSAction::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::PCogX::postSTRIPSAction);
}

::IceProxy::Ice::Object*
IceProxy::PCogX::postSTRIPSAction::__newInstance() const
{
    return new postSTRIPSAction;
}

const ::std::string&
IceProxy::PCogX::postActionDefinition::ice_staticId()
{
    return ::PCogX::postActionDefinition::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::PCogX::postActionDefinition::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::PCogX::postActionDefinition);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::PCogX::postActionDefinition::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::PCogX::postActionDefinition);
}

::IceProxy::Ice::Object*
IceProxy::PCogX::postActionDefinition::__newInstance() const
{
    return new postActionDefinition;
}

const ::std::string&
IceProxy::PCogX::postTypes::ice_staticId()
{
    return ::PCogX::postTypes::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::PCogX::postTypes::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::PCogX::postTypes);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::PCogX::postTypes::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::PCogX::postTypes);
}

::IceProxy::Ice::Object*
IceProxy::PCogX::postTypes::__newInstance() const
{
    return new postTypes;
}

const ::std::string&
IceProxy::PCogX::postXXsubtypeofYY::ice_staticId()
{
    return ::PCogX::postXXsubtypeofYY::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::PCogX::postXXsubtypeofYY::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::PCogX::postXXsubtypeofYY);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::PCogX::postXXsubtypeofYY::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::PCogX::postXXsubtypeofYY);
}

::IceProxy::Ice::Object*
IceProxy::PCogX::postXXsubtypeofYY::__newInstance() const
{
    return new postXXsubtypeofYY;
}

PCogX::obtainPlanner::obtainPlanner(const ::std::string& __ice_identityOfCreatedPlannerIsAReturn, const ::PCogX::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument, const ::PCogX::OptionalMemberPlannerDescriptor& __ice_optionalMemberPlannerDescriptorIsAnArgument) :
    identityOfCreatedPlannerIsAReturn(__ice_identityOfCreatedPlannerIsAReturn),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument),
    optionalMemberPlannerDescriptorIsAnArgument(__ice_optionalMemberPlannerDescriptorIsAnArgument)
{
}

::Ice::ObjectPtr
PCogX::obtainPlanner::ice_clone() const
{
    ::PCogX::obtainPlannerPtr __p = new ::PCogX::obtainPlanner(*this);
    return __p;
}

static const ::std::string __PCogX__obtainPlanner_ids[2] =
{
    "::Ice::Object",
    "::PCogX::obtainPlanner"
};

bool
PCogX::obtainPlanner::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__PCogX__obtainPlanner_ids, __PCogX__obtainPlanner_ids + 2, _s);
}

::std::vector< ::std::string>
PCogX::obtainPlanner::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__PCogX__obtainPlanner_ids[0], &__PCogX__obtainPlanner_ids[2]);
}

const ::std::string&
PCogX::obtainPlanner::ice_id(const ::Ice::Current&) const
{
    return __PCogX__obtainPlanner_ids[1];
}

const ::std::string&
PCogX::obtainPlanner::ice_staticId()
{
    return __PCogX__obtainPlanner_ids[1];
}

void
PCogX::obtainPlanner::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(identityOfCreatedPlannerIsAReturn);
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    if(optionalMemberPlannerDescriptorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberPlannerDescriptorIsAnArgument[0], &optionalMemberPlannerDescriptorIsAnArgument[0] + optionalMemberPlannerDescriptorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
PCogX::obtainPlanner::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(identityOfCreatedPlannerIsAReturn);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->read(optionalMemberPlannerDescriptorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
PCogX::obtainPlanner::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(identityOfCreatedPlannerIsAReturn);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->writeStringSeq(optionalMemberPlannerDescriptorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
PCogX::obtainPlanner::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    identityOfCreatedPlannerIsAReturn = __inS->readString();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    optionalMemberPlannerDescriptorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__PCogX__obtainPlanner : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::PCogX::obtainPlanner::ice_staticId());
        return new ::PCogX::obtainPlanner;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__PCogX__obtainPlanner_Ptr = new __F__PCogX__obtainPlanner;

const ::Ice::ObjectFactoryPtr&
PCogX::obtainPlanner::ice_factory()
{
    return __F__PCogX__obtainPlanner_Ptr;
}

class __F__PCogX__obtainPlanner__Init
{
public:

    __F__PCogX__obtainPlanner__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::PCogX::obtainPlanner::ice_staticId(), ::PCogX::obtainPlanner::ice_factory());
    }

    ~__F__PCogX__obtainPlanner__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::PCogX::obtainPlanner::ice_staticId());
    }
};

static __F__PCogX__obtainPlanner__Init __F__PCogX__obtainPlanner__i;

#ifdef __APPLE__
extern "C" { void __F__PCogX__obtainPlanner__initializer() {} }
#endif

void 
PCogX::__patch__obtainPlannerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::PCogX::obtainPlannerPtr* p = static_cast< ::PCogX::obtainPlannerPtr*>(__addr);
    assert(p);
    *p = ::PCogX::obtainPlannerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::PCogX::obtainPlanner::ice_staticId(), v->ice_id());
    }
}

bool
PCogX::operator==(const ::PCogX::obtainPlanner& l, const ::PCogX::obtainPlanner& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
PCogX::operator<(const ::PCogX::obtainPlanner& l, const ::PCogX::obtainPlanner& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

PCogX::readPropositionIdentifiers::readPropositionIdentifiers(const ::PCogX::PropositionIDs& __ice_propositionIDsIsAReturn, const ::PCogX::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument, const ::PCogX::OptionalMemberDomainDescriptor& __ice_optionalMemberDomainDescriptorIsAnArgument) :
    propositionIDsIsAReturn(__ice_propositionIDsIsAReturn),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument),
    optionalMemberDomainDescriptorIsAnArgument(__ice_optionalMemberDomainDescriptorIsAnArgument)
{
}

::Ice::ObjectPtr
PCogX::readPropositionIdentifiers::ice_clone() const
{
    ::PCogX::readPropositionIdentifiersPtr __p = new ::PCogX::readPropositionIdentifiers(*this);
    return __p;
}

static const ::std::string __PCogX__readPropositionIdentifiers_ids[2] =
{
    "::Ice::Object",
    "::PCogX::readPropositionIdentifiers"
};

bool
PCogX::readPropositionIdentifiers::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__PCogX__readPropositionIdentifiers_ids, __PCogX__readPropositionIdentifiers_ids + 2, _s);
}

::std::vector< ::std::string>
PCogX::readPropositionIdentifiers::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__PCogX__readPropositionIdentifiers_ids[0], &__PCogX__readPropositionIdentifiers_ids[2]);
}

const ::std::string&
PCogX::readPropositionIdentifiers::ice_id(const ::Ice::Current&) const
{
    return __PCogX__readPropositionIdentifiers_ids[1];
}

const ::std::string&
PCogX::readPropositionIdentifiers::ice_staticId()
{
    return __PCogX__readPropositionIdentifiers_ids[1];
}

void
PCogX::readPropositionIdentifiers::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    ::PCogX::__writePropositionIDs(__os, propositionIDsIsAReturn);
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    if(optionalMemberDomainDescriptorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDomainDescriptorIsAnArgument[0], &optionalMemberDomainDescriptorIsAnArgument[0] + optionalMemberDomainDescriptorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
PCogX::readPropositionIdentifiers::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    ::PCogX::__readPropositionIDs(__is, propositionIDsIsAReturn);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->read(optionalMemberDomainDescriptorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
PCogX::readPropositionIdentifiers::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    ::PCogX::ice_writePropositionIDs(__outS, propositionIDsIsAReturn);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->writeStringSeq(optionalMemberDomainDescriptorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
PCogX::readPropositionIdentifiers::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    ::PCogX::ice_readPropositionIDs(__inS, propositionIDsIsAReturn);
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    optionalMemberDomainDescriptorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__PCogX__readPropositionIdentifiers : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::PCogX::readPropositionIdentifiers::ice_staticId());
        return new ::PCogX::readPropositionIdentifiers;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__PCogX__readPropositionIdentifiers_Ptr = new __F__PCogX__readPropositionIdentifiers;

const ::Ice::ObjectFactoryPtr&
PCogX::readPropositionIdentifiers::ice_factory()
{
    return __F__PCogX__readPropositionIdentifiers_Ptr;
}

class __F__PCogX__readPropositionIdentifiers__Init
{
public:

    __F__PCogX__readPropositionIdentifiers__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::PCogX::readPropositionIdentifiers::ice_staticId(), ::PCogX::readPropositionIdentifiers::ice_factory());
    }

    ~__F__PCogX__readPropositionIdentifiers__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::PCogX::readPropositionIdentifiers::ice_staticId());
    }
};

static __F__PCogX__readPropositionIdentifiers__Init __F__PCogX__readPropositionIdentifiers__i;

#ifdef __APPLE__
extern "C" { void __F__PCogX__readPropositionIdentifiers__initializer() {} }
#endif

void 
PCogX::__patch__readPropositionIdentifiersPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::PCogX::readPropositionIdentifiersPtr* p = static_cast< ::PCogX::readPropositionIdentifiersPtr*>(__addr);
    assert(p);
    *p = ::PCogX::readPropositionIdentifiersPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::PCogX::readPropositionIdentifiers::ice_staticId(), v->ice_id());
    }
}

bool
PCogX::operator==(const ::PCogX::readPropositionIdentifiers& l, const ::PCogX::readPropositionIdentifiers& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
PCogX::operator<(const ::PCogX::readPropositionIdentifiers& l, const ::PCogX::readPropositionIdentifiers& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

PCogX::postSTRIPSAction::postSTRIPSAction(const ::PCogX::AddList& __ice_addListIsAnArgument, const ::PCogX::DeleteList& __ice_deleteListIsAnArgument, const ::PCogX::Precondition& __ice_preconditionIsAnArgument, const ::PCogX::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument, const ::PCogX::OptionalMemberDomainDescriptor& __ice_optionalMemberDomainDescriptorIsAnArgument) :
    addListIsAnArgument(__ice_addListIsAnArgument),
    deleteListIsAnArgument(__ice_deleteListIsAnArgument),
    preconditionIsAnArgument(__ice_preconditionIsAnArgument),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument),
    optionalMemberDomainDescriptorIsAnArgument(__ice_optionalMemberDomainDescriptorIsAnArgument)
{
}

::Ice::ObjectPtr
PCogX::postSTRIPSAction::ice_clone() const
{
    ::PCogX::postSTRIPSActionPtr __p = new ::PCogX::postSTRIPSAction(*this);
    return __p;
}

static const ::std::string __PCogX__postSTRIPSAction_ids[2] =
{
    "::Ice::Object",
    "::PCogX::postSTRIPSAction"
};

bool
PCogX::postSTRIPSAction::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__PCogX__postSTRIPSAction_ids, __PCogX__postSTRIPSAction_ids + 2, _s);
}

::std::vector< ::std::string>
PCogX::postSTRIPSAction::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__PCogX__postSTRIPSAction_ids[0], &__PCogX__postSTRIPSAction_ids[2]);
}

const ::std::string&
PCogX::postSTRIPSAction::ice_id(const ::Ice::Current&) const
{
    return __PCogX__postSTRIPSAction_ids[1];
}

const ::std::string&
PCogX::postSTRIPSAction::ice_staticId()
{
    return __PCogX__postSTRIPSAction_ids[1];
}

void
PCogX::postSTRIPSAction::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    if(addListIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&addListIsAnArgument[0], &addListIsAnArgument[0] + addListIsAnArgument.size());
    }
    if(deleteListIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&deleteListIsAnArgument[0], &deleteListIsAnArgument[0] + deleteListIsAnArgument.size());
    }
    if(preconditionIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&preconditionIsAnArgument[0], &preconditionIsAnArgument[0] + preconditionIsAnArgument.size());
    }
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    if(optionalMemberDomainDescriptorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDomainDescriptorIsAnArgument[0], &optionalMemberDomainDescriptorIsAnArgument[0] + optionalMemberDomainDescriptorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
PCogX::postSTRIPSAction::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(addListIsAnArgument);
    __is->read(deleteListIsAnArgument);
    __is->read(preconditionIsAnArgument);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->read(optionalMemberDomainDescriptorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
PCogX::postSTRIPSAction::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeStringSeq(addListIsAnArgument);
    __outS->writeStringSeq(deleteListIsAnArgument);
    __outS->writeStringSeq(preconditionIsAnArgument);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->writeStringSeq(optionalMemberDomainDescriptorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
PCogX::postSTRIPSAction::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    addListIsAnArgument = __inS->readStringSeq();
    deleteListIsAnArgument = __inS->readStringSeq();
    preconditionIsAnArgument = __inS->readStringSeq();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    optionalMemberDomainDescriptorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__PCogX__postSTRIPSAction : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::PCogX::postSTRIPSAction::ice_staticId());
        return new ::PCogX::postSTRIPSAction;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__PCogX__postSTRIPSAction_Ptr = new __F__PCogX__postSTRIPSAction;

const ::Ice::ObjectFactoryPtr&
PCogX::postSTRIPSAction::ice_factory()
{
    return __F__PCogX__postSTRIPSAction_Ptr;
}

class __F__PCogX__postSTRIPSAction__Init
{
public:

    __F__PCogX__postSTRIPSAction__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::PCogX::postSTRIPSAction::ice_staticId(), ::PCogX::postSTRIPSAction::ice_factory());
    }

    ~__F__PCogX__postSTRIPSAction__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::PCogX::postSTRIPSAction::ice_staticId());
    }
};

static __F__PCogX__postSTRIPSAction__Init __F__PCogX__postSTRIPSAction__i;

#ifdef __APPLE__
extern "C" { void __F__PCogX__postSTRIPSAction__initializer() {} }
#endif

void 
PCogX::__patch__postSTRIPSActionPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::PCogX::postSTRIPSActionPtr* p = static_cast< ::PCogX::postSTRIPSActionPtr*>(__addr);
    assert(p);
    *p = ::PCogX::postSTRIPSActionPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::PCogX::postSTRIPSAction::ice_staticId(), v->ice_id());
    }
}

bool
PCogX::operator==(const ::PCogX::postSTRIPSAction& l, const ::PCogX::postSTRIPSAction& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
PCogX::operator<(const ::PCogX::postSTRIPSAction& l, const ::PCogX::postSTRIPSAction& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

PCogX::postActionDefinition::postActionDefinition(const ::std::string& __ice_actionDefinitionIsAnArgument, bool __ice_successIsAReturn, const ::std::string& __ice_parserComplaintIsAReturn, const ::PCogX::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument, const ::PCogX::OptionalMemberDomainDescriptor& __ice_optionalMemberDomainDescriptorIsAnArgument) :
    actionDefinitionIsAnArgument(__ice_actionDefinitionIsAnArgument),
    successIsAReturn(__ice_successIsAReturn),
    parserComplaintIsAReturn(__ice_parserComplaintIsAReturn),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument),
    optionalMemberDomainDescriptorIsAnArgument(__ice_optionalMemberDomainDescriptorIsAnArgument)
{
}

::Ice::ObjectPtr
PCogX::postActionDefinition::ice_clone() const
{
    ::PCogX::postActionDefinitionPtr __p = new ::PCogX::postActionDefinition(*this);
    return __p;
}

static const ::std::string __PCogX__postActionDefinition_ids[2] =
{
    "::Ice::Object",
    "::PCogX::postActionDefinition"
};

bool
PCogX::postActionDefinition::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__PCogX__postActionDefinition_ids, __PCogX__postActionDefinition_ids + 2, _s);
}

::std::vector< ::std::string>
PCogX::postActionDefinition::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__PCogX__postActionDefinition_ids[0], &__PCogX__postActionDefinition_ids[2]);
}

const ::std::string&
PCogX::postActionDefinition::ice_id(const ::Ice::Current&) const
{
    return __PCogX__postActionDefinition_ids[1];
}

const ::std::string&
PCogX::postActionDefinition::ice_staticId()
{
    return __PCogX__postActionDefinition_ids[1];
}

void
PCogX::postActionDefinition::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(actionDefinitionIsAnArgument);
    __os->write(successIsAReturn);
    __os->write(parserComplaintIsAReturn);
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    if(optionalMemberDomainDescriptorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDomainDescriptorIsAnArgument[0], &optionalMemberDomainDescriptorIsAnArgument[0] + optionalMemberDomainDescriptorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
PCogX::postActionDefinition::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(actionDefinitionIsAnArgument);
    __is->read(successIsAReturn);
    __is->read(parserComplaintIsAReturn);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->read(optionalMemberDomainDescriptorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
PCogX::postActionDefinition::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(actionDefinitionIsAnArgument);
    __outS->writeBool(successIsAReturn);
    __outS->writeString(parserComplaintIsAReturn);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->writeStringSeq(optionalMemberDomainDescriptorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
PCogX::postActionDefinition::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    actionDefinitionIsAnArgument = __inS->readString();
    successIsAReturn = __inS->readBool();
    parserComplaintIsAReturn = __inS->readString();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    optionalMemberDomainDescriptorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__PCogX__postActionDefinition : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::PCogX::postActionDefinition::ice_staticId());
        return new ::PCogX::postActionDefinition;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__PCogX__postActionDefinition_Ptr = new __F__PCogX__postActionDefinition;

const ::Ice::ObjectFactoryPtr&
PCogX::postActionDefinition::ice_factory()
{
    return __F__PCogX__postActionDefinition_Ptr;
}

class __F__PCogX__postActionDefinition__Init
{
public:

    __F__PCogX__postActionDefinition__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::PCogX::postActionDefinition::ice_staticId(), ::PCogX::postActionDefinition::ice_factory());
    }

    ~__F__PCogX__postActionDefinition__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::PCogX::postActionDefinition::ice_staticId());
    }
};

static __F__PCogX__postActionDefinition__Init __F__PCogX__postActionDefinition__i;

#ifdef __APPLE__
extern "C" { void __F__PCogX__postActionDefinition__initializer() {} }
#endif

void 
PCogX::__patch__postActionDefinitionPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::PCogX::postActionDefinitionPtr* p = static_cast< ::PCogX::postActionDefinitionPtr*>(__addr);
    assert(p);
    *p = ::PCogX::postActionDefinitionPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::PCogX::postActionDefinition::ice_staticId(), v->ice_id());
    }
}

bool
PCogX::operator==(const ::PCogX::postActionDefinition& l, const ::PCogX::postActionDefinition& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
PCogX::operator<(const ::PCogX::postActionDefinition& l, const ::PCogX::postActionDefinition& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

PCogX::postTypes::postTypes(const ::std::string& __ice_typesDefinitionIsAnArgument, bool __ice_successIsAReturn, const ::std::string& __ice_ParserComplaintIsAReturn, const ::PCogX::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument, const ::PCogX::OptionalMemberDomainDescriptor& __ice_optionalMemberDomainDescriptorIsAnArgument) :
    typesDefinitionIsAnArgument(__ice_typesDefinitionIsAnArgument),
    successIsAReturn(__ice_successIsAReturn),
    ParserComplaintIsAReturn(__ice_ParserComplaintIsAReturn),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument),
    optionalMemberDomainDescriptorIsAnArgument(__ice_optionalMemberDomainDescriptorIsAnArgument)
{
}

::Ice::ObjectPtr
PCogX::postTypes::ice_clone() const
{
    ::PCogX::postTypesPtr __p = new ::PCogX::postTypes(*this);
    return __p;
}

static const ::std::string __PCogX__postTypes_ids[2] =
{
    "::Ice::Object",
    "::PCogX::postTypes"
};

bool
PCogX::postTypes::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__PCogX__postTypes_ids, __PCogX__postTypes_ids + 2, _s);
}

::std::vector< ::std::string>
PCogX::postTypes::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__PCogX__postTypes_ids[0], &__PCogX__postTypes_ids[2]);
}

const ::std::string&
PCogX::postTypes::ice_id(const ::Ice::Current&) const
{
    return __PCogX__postTypes_ids[1];
}

const ::std::string&
PCogX::postTypes::ice_staticId()
{
    return __PCogX__postTypes_ids[1];
}

void
PCogX::postTypes::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(typesDefinitionIsAnArgument);
    __os->write(successIsAReturn);
    __os->write(ParserComplaintIsAReturn);
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    if(optionalMemberDomainDescriptorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDomainDescriptorIsAnArgument[0], &optionalMemberDomainDescriptorIsAnArgument[0] + optionalMemberDomainDescriptorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
PCogX::postTypes::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(typesDefinitionIsAnArgument);
    __is->read(successIsAReturn);
    __is->read(ParserComplaintIsAReturn);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->read(optionalMemberDomainDescriptorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
PCogX::postTypes::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeString(typesDefinitionIsAnArgument);
    __outS->writeBool(successIsAReturn);
    __outS->writeString(ParserComplaintIsAReturn);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->writeStringSeq(optionalMemberDomainDescriptorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
PCogX::postTypes::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    typesDefinitionIsAnArgument = __inS->readString();
    successIsAReturn = __inS->readBool();
    ParserComplaintIsAReturn = __inS->readString();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    optionalMemberDomainDescriptorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__PCogX__postTypes : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::PCogX::postTypes::ice_staticId());
        return new ::PCogX::postTypes;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__PCogX__postTypes_Ptr = new __F__PCogX__postTypes;

const ::Ice::ObjectFactoryPtr&
PCogX::postTypes::ice_factory()
{
    return __F__PCogX__postTypes_Ptr;
}

class __F__PCogX__postTypes__Init
{
public:

    __F__PCogX__postTypes__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::PCogX::postTypes::ice_staticId(), ::PCogX::postTypes::ice_factory());
    }

    ~__F__PCogX__postTypes__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::PCogX::postTypes::ice_staticId());
    }
};

static __F__PCogX__postTypes__Init __F__PCogX__postTypes__i;

#ifdef __APPLE__
extern "C" { void __F__PCogX__postTypes__initializer() {} }
#endif

void 
PCogX::__patch__postTypesPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::PCogX::postTypesPtr* p = static_cast< ::PCogX::postTypesPtr*>(__addr);
    assert(p);
    *p = ::PCogX::postTypesPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::PCogX::postTypes::ice_staticId(), v->ice_id());
    }
}

bool
PCogX::operator==(const ::PCogX::postTypes& l, const ::PCogX::postTypes& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
PCogX::operator<(const ::PCogX::postTypes& l, const ::PCogX::postTypes& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

PCogX::postXXsubtypeofYY::postXXsubtypeofYY(const ::PCogX::XX& __ice_XXIsAnArgument, const ::PCogX::YY& __ice_YYIsAnArgument, const ::PCogX::OptionalMemberDesignator& __ice_optionalMemberDesignatorIsAnArgument, const ::PCogX::OptionalMemberDomainDescriptor& __ice_optionalMemberDomainDescriptorIsAnArgument) :
    XXIsAnArgument(__ice_XXIsAnArgument),
    YYIsAnArgument(__ice_YYIsAnArgument),
    optionalMemberDesignatorIsAnArgument(__ice_optionalMemberDesignatorIsAnArgument),
    optionalMemberDomainDescriptorIsAnArgument(__ice_optionalMemberDomainDescriptorIsAnArgument)
{
}

::Ice::ObjectPtr
PCogX::postXXsubtypeofYY::ice_clone() const
{
    ::PCogX::postXXsubtypeofYYPtr __p = new ::PCogX::postXXsubtypeofYY(*this);
    return __p;
}

static const ::std::string __PCogX__postXXsubtypeofYY_ids[2] =
{
    "::Ice::Object",
    "::PCogX::postXXsubtypeofYY"
};

bool
PCogX::postXXsubtypeofYY::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__PCogX__postXXsubtypeofYY_ids, __PCogX__postXXsubtypeofYY_ids + 2, _s);
}

::std::vector< ::std::string>
PCogX::postXXsubtypeofYY::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__PCogX__postXXsubtypeofYY_ids[0], &__PCogX__postXXsubtypeofYY_ids[2]);
}

const ::std::string&
PCogX::postXXsubtypeofYY::ice_id(const ::Ice::Current&) const
{
    return __PCogX__postXXsubtypeofYY_ids[1];
}

const ::std::string&
PCogX::postXXsubtypeofYY::ice_staticId()
{
    return __PCogX__postXXsubtypeofYY_ids[1];
}

void
PCogX::postXXsubtypeofYY::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    if(XXIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&XXIsAnArgument[0], &XXIsAnArgument[0] + XXIsAnArgument.size());
    }
    if(YYIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&YYIsAnArgument[0], &YYIsAnArgument[0] + YYIsAnArgument.size());
    }
    if(optionalMemberDesignatorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDesignatorIsAnArgument[0], &optionalMemberDesignatorIsAnArgument[0] + optionalMemberDesignatorIsAnArgument.size());
    }
    if(optionalMemberDomainDescriptorIsAnArgument.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&optionalMemberDomainDescriptorIsAnArgument[0], &optionalMemberDomainDescriptorIsAnArgument[0] + optionalMemberDomainDescriptorIsAnArgument.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
PCogX::postXXsubtypeofYY::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(XXIsAnArgument);
    __is->read(YYIsAnArgument);
    __is->read(optionalMemberDesignatorIsAnArgument);
    __is->read(optionalMemberDomainDescriptorIsAnArgument);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
PCogX::postXXsubtypeofYY::__write(const ::Ice::OutputStreamPtr& __outS) const
{
    __outS->writeTypeId(ice_staticId());
    __outS->startSlice();
    __outS->writeStringSeq(XXIsAnArgument);
    __outS->writeStringSeq(YYIsAnArgument);
    __outS->writeStringSeq(optionalMemberDesignatorIsAnArgument);
    __outS->writeStringSeq(optionalMemberDomainDescriptorIsAnArgument);
    __outS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__outS);
#else
    ::Ice::Object::__write(__outS);
#endif
}

void
PCogX::postXXsubtypeofYY::__read(const ::Ice::InputStreamPtr& __inS, bool __rid)
{
    if(__rid)
    {
        __inS->readTypeId();
    }
    __inS->startSlice();
    XXIsAnArgument = __inS->readStringSeq();
    YYIsAnArgument = __inS->readStringSeq();
    optionalMemberDesignatorIsAnArgument = __inS->readStringSeq();
    optionalMemberDomainDescriptorIsAnArgument = __inS->readStringSeq();
    __inS->endSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__inS, true);
#else
    ::Ice::Object::__read(__inS, true);
#endif
}

class __F__PCogX__postXXsubtypeofYY : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::PCogX::postXXsubtypeofYY::ice_staticId());
        return new ::PCogX::postXXsubtypeofYY;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__PCogX__postXXsubtypeofYY_Ptr = new __F__PCogX__postXXsubtypeofYY;

const ::Ice::ObjectFactoryPtr&
PCogX::postXXsubtypeofYY::ice_factory()
{
    return __F__PCogX__postXXsubtypeofYY_Ptr;
}

class __F__PCogX__postXXsubtypeofYY__Init
{
public:

    __F__PCogX__postXXsubtypeofYY__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::PCogX::postXXsubtypeofYY::ice_staticId(), ::PCogX::postXXsubtypeofYY::ice_factory());
    }

    ~__F__PCogX__postXXsubtypeofYY__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::PCogX::postXXsubtypeofYY::ice_staticId());
    }
};

static __F__PCogX__postXXsubtypeofYY__Init __F__PCogX__postXXsubtypeofYY__i;

#ifdef __APPLE__
extern "C" { void __F__PCogX__postXXsubtypeofYY__initializer() {} }
#endif

void 
PCogX::__patch__postXXsubtypeofYYPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::PCogX::postXXsubtypeofYYPtr* p = static_cast< ::PCogX::postXXsubtypeofYYPtr*>(__addr);
    assert(p);
    *p = ::PCogX::postXXsubtypeofYYPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::PCogX::postXXsubtypeofYY::ice_staticId(), v->ice_id());
    }
}

bool
PCogX::operator==(const ::PCogX::postXXsubtypeofYY& l, const ::PCogX::postXXsubtypeofYY& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
PCogX::operator<(const ::PCogX::postXXsubtypeofYY& l, const ::PCogX::postXXsubtypeofYY& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
