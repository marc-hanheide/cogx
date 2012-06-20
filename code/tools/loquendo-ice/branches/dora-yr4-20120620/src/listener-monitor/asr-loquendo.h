// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `asr-loquendo.ice'

#ifndef __asr_loquendo_h__
#define __asr_loquendo_h__

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
#include <Ice/Identity.h>
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

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal;

}

namespace result
{

class RecognitionResult;

class NoRecognitionResult;

class WordHypothesis;

class Hypothesis;

class NBestList;

class Node;

class Link;

class WordLattice;

}

class AudioSource;

class PulseAudioPCMCapture;

class RAWFile;

class Server;

class Client;

}

}

}

}

}

}

}

}

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal;
bool operator==(const TimeVal&, const TimeVal&);
bool operator<(const TimeVal&, const TimeVal&);

}

namespace result
{

class RecognitionResult;
bool operator==(const RecognitionResult&, const RecognitionResult&);
bool operator<(const RecognitionResult&, const RecognitionResult&);

class NoRecognitionResult;
bool operator==(const NoRecognitionResult&, const NoRecognitionResult&);
bool operator<(const NoRecognitionResult&, const NoRecognitionResult&);

class WordHypothesis;
bool operator==(const WordHypothesis&, const WordHypothesis&);
bool operator<(const WordHypothesis&, const WordHypothesis&);

class Hypothesis;
bool operator==(const Hypothesis&, const Hypothesis&);
bool operator<(const Hypothesis&, const Hypothesis&);

class NBestList;
bool operator==(const NBestList&, const NBestList&);
bool operator<(const NBestList&, const NBestList&);

class Node;
bool operator==(const Node&, const Node&);
bool operator<(const Node&, const Node&);

class Link;
bool operator==(const Link&, const Link&);
bool operator<(const Link&, const Link&);

class WordLattice;
bool operator==(const WordLattice&, const WordLattice&);
bool operator<(const WordLattice&, const WordLattice&);

}

class AudioSource;
bool operator==(const AudioSource&, const AudioSource&);
bool operator<(const AudioSource&, const AudioSource&);

class PulseAudioPCMCapture;
bool operator==(const PulseAudioPCMCapture&, const PulseAudioPCMCapture&);
bool operator<(const PulseAudioPCMCapture&, const PulseAudioPCMCapture&);

class RAWFile;
bool operator==(const RAWFile&, const RAWFile&);
bool operator<(const RAWFile&, const RAWFile&);

class Server;
bool operator==(const Server&, const Server&);
bool operator<(const Server&, const Server&);

class Client;
bool operator==(const Client&, const Client&);
bool operator<(const Client&, const Client&);

}

}

}

}

}

}

}

namespace IceInternal
{

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::Server*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server*);

::Ice::Object* upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::Client*);
::IceProxy::Ice::Object* upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client*);

}

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal> TimeValPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal> TimeValPrx;

void __read(::IceInternal::BasicStream*, TimeValPrx&);
void __patch__TimeValPtr(void*, ::Ice::ObjectPtr&);

}

namespace result
{

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult> RecognitionResultPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult> RecognitionResultPrx;

void __read(::IceInternal::BasicStream*, RecognitionResultPrx&);
void __patch__RecognitionResultPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult> NoRecognitionResultPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult> NoRecognitionResultPrx;

void __read(::IceInternal::BasicStream*, NoRecognitionResultPrx&);
void __patch__NoRecognitionResultPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis> WordHypothesisPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis> WordHypothesisPrx;

void __read(::IceInternal::BasicStream*, WordHypothesisPrx&);
void __patch__WordHypothesisPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis> HypothesisPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis> HypothesisPrx;

void __read(::IceInternal::BasicStream*, HypothesisPrx&);
void __patch__HypothesisPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList> NBestListPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList> NBestListPrx;

void __read(::IceInternal::BasicStream*, NBestListPrx&);
void __patch__NBestListPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node> NodePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node> NodePrx;

void __read(::IceInternal::BasicStream*, NodePrx&);
void __patch__NodePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link> LinkPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link> LinkPrx;

void __read(::IceInternal::BasicStream*, LinkPrx&);
void __patch__LinkPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice> WordLatticePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice> WordLatticePrx;

void __read(::IceInternal::BasicStream*, WordLatticePrx&);
void __patch__WordLatticePtr(void*, ::Ice::ObjectPtr&);

}

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource> AudioSourcePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource> AudioSourcePrx;

void __read(::IceInternal::BasicStream*, AudioSourcePrx&);
void __patch__AudioSourcePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture> PulseAudioPCMCapturePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture> PulseAudioPCMCapturePrx;

void __read(::IceInternal::BasicStream*, PulseAudioPCMCapturePrx&);
void __patch__PulseAudioPCMCapturePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile> RAWFilePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile> RAWFilePrx;

void __read(::IceInternal::BasicStream*, RAWFilePrx&);
void __patch__RAWFilePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server> ServerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server> ServerPrx;

void __read(::IceInternal::BasicStream*, ServerPrx&);
void __patch__ServerPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client> ClientPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client> ClientPrx;

void __read(::IceInternal::BasicStream*, ClientPrx&);
void __patch__ClientPtr(void*, ::Ice::ObjectPtr&);

}

}

}

}

}

}

}

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

const ::std::string RELEASE = "1.2";

namespace result
{

typedef ::std::vector< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr> WordHypothesisSeq;
void __writeWordHypothesisSeq(::IceInternal::BasicStream*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr*);
void __readWordHypothesisSeq(::IceInternal::BasicStream*, WordHypothesisSeq&);

typedef ::std::vector< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr> HypothesisSeq;
void __writeHypothesisSeq(::IceInternal::BasicStream*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr*);
void __readHypothesisSeq(::IceInternal::BasicStream*, HypothesisSeq&);

enum RejectionFlag
{
    RecognitionFalse,
    RecognitionNomatch
};

void __write(::IceInternal::BasicStream*, RejectionFlag);
void __read(::IceInternal::BasicStream*, RejectionFlag&);

typedef ::std::vector< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr> NodeSeq;
void __writeNodeSeq(::IceInternal::BasicStream*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr*);
void __readNodeSeq(::IceInternal::BasicStream*, NodeSeq&);

typedef ::std::vector< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr> LinkSeq;
void __writeLinkSeq(::IceInternal::BasicStream*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr*, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr*);
void __readLinkSeq(::IceInternal::BasicStream*, LinkSeq&);

}

class LoquendoException : public ::Ice::UserException
{
public:

    LoquendoException() {}
    LoquendoException(const ::std::string&, ::Ice::Int, const ::std::string&);
    virtual ~LoquendoException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string function;
    ::Ice::Int returnCode;
    ::std::string errorMessage;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

static LoquendoException __LoquendoException_init;

class ServerException : public ::Ice::UserException
{
public:

    ServerException() {}
    explicit ServerException(const ::std::string&);
    virtual ~ServerException() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string message;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

}

}

}

}

}

}

namespace IceProxy
{

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<TimeVal> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<TimeVal> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<TimeVal*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<TimeVal*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

namespace result
{

class RecognitionResult : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RecognitionResult> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RecognitionResult*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<RecognitionResult*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class NoRecognitionResult : virtual public ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NoRecognitionResult> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NoRecognitionResult*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<NoRecognitionResult*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class WordHypothesis : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordHypothesis> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordHypothesis*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<WordHypothesis*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Hypothesis : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Hypothesis> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Hypothesis*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Hypothesis*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class NBestList : virtual public ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
    
    ::IceInternal::ProxyHandle<NBestList> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<NBestList> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<NBestList*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<NBestList*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Node : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Node> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Node> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Node*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Node*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Link : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<Link> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Link> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Link*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Link*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class WordLattice : virtual public ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
    
    ::IceInternal::ProxyHandle<WordLattice> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<WordLattice> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<WordLattice*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<WordLattice*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

class AudioSource : virtual public ::IceProxy::Ice::Object
{
public:
    
    ::IceInternal::ProxyHandle<AudioSource> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<AudioSource> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<AudioSource*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<AudioSource*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class PulseAudioPCMCapture : virtual public ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<PulseAudioPCMCapture> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<PulseAudioPCMCapture*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<PulseAudioPCMCapture*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class RAWFile : virtual public ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
    
    ::IceInternal::ProxyHandle<RAWFile> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<RAWFile> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RAWFile*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<RAWFile*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Server : virtual public ::IceProxy::Ice::Object
{
public:

    void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as)
    {
        setAudioSource(as, 0);
    }
    void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context& __ctx)
    {
        setAudioSource(as, &__ctx);
    }
    
private:

    void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*);
    
public:

    void setGrammarFile(const ::std::string& filename)
    {
        setGrammarFile(filename, 0);
    }
    void setGrammarFile(const ::std::string& filename, const ::Ice::Context& __ctx)
    {
        setGrammarFile(filename, &__ctx);
    }
    
private:

    void setGrammarFile(const ::std::string&, const ::Ice::Context*);
    
public:

    void addClient(const ::Ice::Identity& ident)
    {
        addClient(ident, 0);
    }
    void addClient(const ::Ice::Identity& ident, const ::Ice::Context& __ctx)
    {
        addClient(ident, &__ctx);
    }
    
private:

    void addClient(const ::Ice::Identity&, const ::Ice::Context*);
    
public:

    void start()
    {
        start(0);
    }
    void start(const ::Ice::Context& __ctx)
    {
        start(&__ctx);
    }
    
private:

    void start(const ::Ice::Context*);
    
public:

    void stop()
    {
        stop(0);
    }
    void stop(const ::Ice::Context& __ctx)
    {
        stop(&__ctx);
    }
    
private:

    void stop(const ::Ice::Context*);
    
public:

    void shutdown()
    {
        shutdown(0);
    }
    void shutdown(const ::Ice::Context& __ctx)
    {
        shutdown(&__ctx);
    }
    
private:

    void shutdown(const ::Ice::Context*);
    
public:

    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime()
    {
        return getCurrentTime(0);
    }
    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime(const ::Ice::Context& __ctx)
    {
        return getCurrentTime(&__ctx);
    }
    
private:

    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime(const ::Ice::Context*);
    
public:

    ::std::string getCWD()
    {
        return getCWD(0);
    }
    ::std::string getCWD(const ::Ice::Context& __ctx)
    {
        return getCWD(&__ctx);
    }
    
private:

    ::std::string getCWD(const ::Ice::Context*);
    
public:

    ::std::string getStatusString()
    {
        return getStatusString(0);
    }
    ::std::string getStatusString(const ::Ice::Context& __ctx)
    {
        return getStatusString(&__ctx);
    }
    
private:

    ::std::string getStatusString(const ::Ice::Context*);
    
public:

    ::Ice::IdentitySeq getClients()
    {
        return getClients(0);
    }
    ::Ice::IdentitySeq getClients(const ::Ice::Context& __ctx)
    {
        return getClients(&__ctx);
    }
    
private:

    ::Ice::IdentitySeq getClients(const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<Server> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Server> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Server*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Server*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Client : virtual public ::IceProxy::Ice::Object
{
public:

    void onStart()
    {
        onStart(0);
    }
    void onStart(const ::Ice::Context& __ctx)
    {
        onStart(&__ctx);
    }
    
private:

    void onStart(const ::Ice::Context*);
    
public:

    void onStop()
    {
        onStop(0);
    }
    void onStop(const ::Ice::Context& __ctx)
    {
        onStop(&__ctx);
    }
    
private:

    void onStop(const ::Ice::Context*);
    
public:

    void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as)
    {
        onAudioSourceChange(as, 0);
    }
    void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context& __ctx)
    {
        onAudioSourceChange(as, &__ctx);
    }
    
private:

    void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*);
    
public:

    void onEndOfStream()
    {
        onEndOfStream(0);
    }
    void onEndOfStream(const ::Ice::Context& __ctx)
    {
        onEndOfStream(&__ctx);
    }
    
private:

    void onEndOfStream(const ::Ice::Context*);
    
public:

    void onGrammarChange(const ::std::string& filename)
    {
        onGrammarChange(filename, 0);
    }
    void onGrammarChange(const ::std::string& filename, const ::Ice::Context& __ctx)
    {
        onGrammarChange(filename, &__ctx);
    }
    
private:

    void onGrammarChange(const ::std::string&, const ::Ice::Context*);
    
public:

    void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& res)
    {
        onRecognitionResult(res, 0);
    }
    void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& res, const ::Ice::Context& __ctx)
    {
        onRecognitionResult(res, &__ctx);
    }
    
private:

    void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr&, const ::Ice::Context*);
    
public:

    void onUnregistrationFromServer(const ::std::string& reason)
    {
        onUnregistrationFromServer(reason, 0);
    }
    void onUnregistrationFromServer(const ::std::string& reason, const ::Ice::Context& __ctx)
    {
        onUnregistrationFromServer(reason, &__ctx);
    }
    
private:

    void onUnregistrationFromServer(const ::std::string&, const ::Ice::Context*);
    
public:
    
    ::IceInternal::ProxyHandle<Client> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }
    
    ::IceInternal::ProxyHandle<Client> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Client*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Client*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

}

}

}

}

}

namespace IceDelegate
{

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal : virtual public ::IceDelegate::Ice::Object
{
public:
};

}

namespace result
{

class RecognitionResult : virtual public ::IceDelegate::Ice::Object
{
public:
};

class NoRecognitionResult : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

class WordHypothesis : virtual public ::IceDelegate::Ice::Object
{
public:
};

class Hypothesis : virtual public ::IceDelegate::Ice::Object
{
public:
};

class NBestList : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

class Node : virtual public ::IceDelegate::Ice::Object
{
public:
};

class Link : virtual public ::IceDelegate::Ice::Object
{
public:
};

class WordLattice : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

}

class AudioSource : virtual public ::IceDelegate::Ice::Object
{
public:
};

class PulseAudioPCMCapture : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
};

class RAWFile : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
};

class Server : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*) = 0;

    virtual void setGrammarFile(const ::std::string&, const ::Ice::Context*) = 0;

    virtual void addClient(const ::Ice::Identity&, const ::Ice::Context*) = 0;

    virtual void start(const ::Ice::Context*) = 0;

    virtual void stop(const ::Ice::Context*) = 0;

    virtual void shutdown(const ::Ice::Context*) = 0;

    virtual ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime(const ::Ice::Context*) = 0;

    virtual ::std::string getCWD(const ::Ice::Context*) = 0;

    virtual ::std::string getStatusString(const ::Ice::Context*) = 0;

    virtual ::Ice::IdentitySeq getClients(const ::Ice::Context*) = 0;
};

class Client : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void onStart(const ::Ice::Context*) = 0;

    virtual void onStop(const ::Ice::Context*) = 0;

    virtual void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*) = 0;

    virtual void onEndOfStream(const ::Ice::Context*) = 0;

    virtual void onGrammarChange(const ::std::string&, const ::Ice::Context*) = 0;

    virtual void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr&, const ::Ice::Context*) = 0;

    virtual void onUnregistrationFromServer(const ::std::string&, const ::Ice::Context*) = 0;
};

}

}

}

}

}

}

}

}

namespace IceDelegateM
{

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal,
                virtual public ::IceDelegateM::Ice::Object
{
public:
};

}

namespace result
{

class RecognitionResult : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult,
                          virtual public ::IceDelegateM::Ice::Object
{
public:
};

class NoRecognitionResult : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult,
                            virtual public ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

class WordHypothesis : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis,
                       virtual public ::IceDelegateM::Ice::Object
{
public:
};

class Hypothesis : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis,
                   virtual public ::IceDelegateM::Ice::Object
{
public:
};

class NBestList : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList,
                  virtual public ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

class Node : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node,
             virtual public ::IceDelegateM::Ice::Object
{
public:
};

class Link : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link,
             virtual public ::IceDelegateM::Ice::Object
{
public:
};

class WordLattice : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice,
                    virtual public ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

}

class AudioSource : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource,
                    virtual public ::IceDelegateM::Ice::Object
{
public:
};

class PulseAudioPCMCapture : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture,
                             virtual public ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
};

class RAWFile : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile,
                virtual public ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
};

class Server : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server,
               virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*);

    virtual void setGrammarFile(const ::std::string&, const ::Ice::Context*);

    virtual void addClient(const ::Ice::Identity&, const ::Ice::Context*);

    virtual void start(const ::Ice::Context*);

    virtual void stop(const ::Ice::Context*);

    virtual void shutdown(const ::Ice::Context*);

    virtual ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime(const ::Ice::Context*);

    virtual ::std::string getCWD(const ::Ice::Context*);

    virtual ::std::string getStatusString(const ::Ice::Context*);

    virtual ::Ice::IdentitySeq getClients(const ::Ice::Context*);
};

class Client : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client,
               virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void onStart(const ::Ice::Context*);

    virtual void onStop(const ::Ice::Context*);

    virtual void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*);

    virtual void onEndOfStream(const ::Ice::Context*);

    virtual void onGrammarChange(const ::std::string&, const ::Ice::Context*);

    virtual void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr&, const ::Ice::Context*);

    virtual void onUnregistrationFromServer(const ::std::string&, const ::Ice::Context*);
};

}

}

}

}

}

}

}

}

namespace IceDelegateD
{

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal,
                virtual public ::IceDelegateD::Ice::Object
{
public:
};

}

namespace result
{

class RecognitionResult : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult,
                          virtual public ::IceDelegateD::Ice::Object
{
public:
};

class NoRecognitionResult : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult,
                            virtual public ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

class WordHypothesis : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis,
                       virtual public ::IceDelegateD::Ice::Object
{
public:
};

class Hypothesis : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis,
                   virtual public ::IceDelegateD::Ice::Object
{
public:
};

class NBestList : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList,
                  virtual public ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

class Node : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node,
             virtual public ::IceDelegateD::Ice::Object
{
public:
};

class Link : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link,
             virtual public ::IceDelegateD::Ice::Object
{
public:
};

class WordLattice : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice,
                    virtual public ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:
};

}

class AudioSource : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource,
                    virtual public ::IceDelegateD::Ice::Object
{
public:
};

class PulseAudioPCMCapture : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture,
                             virtual public ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
};

class RAWFile : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile,
                virtual public ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:
};

class Server : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server,
               virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*);

    virtual void setGrammarFile(const ::std::string&, const ::Ice::Context*);

    virtual void addClient(const ::Ice::Identity&, const ::Ice::Context*);

    virtual void start(const ::Ice::Context*);

    virtual void stop(const ::Ice::Context*);

    virtual void shutdown(const ::Ice::Context*);

    virtual ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime(const ::Ice::Context*);

    virtual ::std::string getCWD(const ::Ice::Context*);

    virtual ::std::string getStatusString(const ::Ice::Context*);

    virtual ::Ice::IdentitySeq getClients(const ::Ice::Context*);
};

class Client : virtual public ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client,
               virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void onStart(const ::Ice::Context*);

    virtual void onStop(const ::Ice::Context*);

    virtual void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Context*);

    virtual void onEndOfStream(const ::Ice::Context*);

    virtual void onGrammarChange(const ::std::string&, const ::Ice::Context*);

    virtual void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr&, const ::Ice::Context*);

    virtual void onUnregistrationFromServer(const ::std::string&, const ::Ice::Context*);
};

}

}

}

}

}

}

}

}

namespace de
{

namespace dfki
{

namespace lt
{

namespace tr
{

namespace dialogue
{

namespace asr
{

namespace loquendo
{

namespace time
{

class TimeVal : virtual public ::Ice::Object
{
public:

    typedef TimeValPrx ProxyType;
    typedef TimeValPtr PointerType;
    
    TimeVal() {}
    TimeVal(::Ice::Long, ::Ice::Int);
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

    virtual ~TimeVal() {}

    friend class TimeVal__staticInit;

public:

    ::Ice::Long sec;

    ::Ice::Int usec;
};

class TimeVal__staticInit
{
public:

    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal _init;
};

static TimeVal__staticInit _TimeVal_init;

}

namespace result
{

class RecognitionResult : virtual public ::Ice::Object
{
public:

    typedef RecognitionResultPrx ProxyType;
    typedef RecognitionResultPtr PointerType;
    
    RecognitionResult() {}
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

    virtual ~RecognitionResult() {}
};

class NoRecognitionResult : virtual public ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:

    typedef NoRecognitionResultPrx ProxyType;
    typedef NoRecognitionResultPtr PointerType;
    
    NoRecognitionResult() {}
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

    virtual ~NoRecognitionResult() {}
};

class WordHypothesis : virtual public ::Ice::Object
{
public:

    typedef WordHypothesisPrx ProxyType;
    typedef WordHypothesisPtr PointerType;
    
    WordHypothesis() {}
    WordHypothesis(const ::std::string&, const ::std::string&, ::Ice::Float, ::Ice::Float, ::Ice::Int, ::Ice::Int);
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

    virtual ~WordHypothesis() {}

public:

    ::std::string word;

    ::std::string lang;

    ::Ice::Float acousticScore;

    ::Ice::Float confidence;

    ::Ice::Int startFrame;

    ::Ice::Int endFrame;
};

class Hypothesis : virtual public ::Ice::Object
{
public:

    typedef HypothesisPrx ProxyType;
    typedef HypothesisPtr PointerType;
    
    Hypothesis() {}
    Hypothesis(const ::std::string&, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisSeq&, ::Ice::Float, ::Ice::Float, const ::std::string&, const ::std::string&);
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

    virtual ~Hypothesis() {}

public:

    ::std::string str;

    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisSeq words;

    ::Ice::Float acousticScore;

    ::Ice::Float confidence;

    ::std::string roName;

    ::std::string roRule;
};

class NBestList : virtual public ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:

    typedef NBestListPrx ProxyType;
    typedef NBestListPtr PointerType;
    
    NBestList() {}
    NBestList(const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr&, ::Ice::Float, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RejectionFlag, const ::std::string&, const ::std::string&, ::Ice::Int, ::Ice::Int, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisSeq&);
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

    virtual ~NBestList() {}

public:

    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr timeAnchor;

    ::Ice::Float snr;

    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RejectionFlag rejectionAdvice;

    ::std::string roName;

    ::std::string roRule;

    ::Ice::Int speechStartFrame;

    ::Ice::Int speechEndFrame;

    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisSeq hypos;
};

class Node : virtual public ::Ice::Object
{
public:

    typedef NodePrx ProxyType;
    typedef NodePtr PointerType;
    
    Node() {}
    Node(::Ice::Int, ::Ice::Float, ::Ice::Float);
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

    virtual ~Node() {}

public:

    ::Ice::Int id;

    ::Ice::Float time;

    ::Ice::Float frame;
};

class Link : virtual public ::Ice::Object
{
public:

    typedef LinkPrx ProxyType;
    typedef LinkPtr PointerType;
    
    Link() {}
    Link(::Ice::Int, ::Ice::Int, ::Ice::Int, const ::std::string&, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float);
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

    virtual ~Link() {}

public:

    ::Ice::Int id;

    ::Ice::Int start;

    ::Ice::Int end;

    ::std::string word;

    ::Ice::Float confidence;

    ::Ice::Float searchScore;

    ::Ice::Float sequenceScore;

    ::Ice::Float acoustic;
};

class WordLattice : virtual public ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult
{
public:

    typedef WordLatticePrx ProxyType;
    typedef WordLatticePtr PointerType;
    
    WordLattice() {}
    WordLattice(const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr&, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodeSeq&, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkSeq&);
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

    virtual ~WordLattice() {}

public:

    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr timeAnchor;

    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodeSeq nodes;

    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkSeq links;
};

}

class AudioSource : virtual public ::Ice::Object
{
public:

    typedef AudioSourcePrx ProxyType;
    typedef AudioSourcePtr PointerType;
    
    AudioSource() {}
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

    virtual ~AudioSource() {}
};

class PulseAudioPCMCapture : virtual public ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:

    typedef PulseAudioPCMCapturePrx ProxyType;
    typedef PulseAudioPCMCapturePtr PointerType;
    
    PulseAudioPCMCapture() {}
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

    virtual ~PulseAudioPCMCapture() {}
};

class RAWFile : virtual public ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource
{
public:

    typedef RAWFilePrx ProxyType;
    typedef RAWFilePtr PointerType;
    
    RAWFile() {}
    explicit RAWFile(const ::std::string&);
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

    virtual ~RAWFile() {}

public:

    ::std::string path;
};

class Server : virtual public ::Ice::Object
{
public:

    typedef ServerPrx ProxyType;
    typedef ServerPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setAudioSource(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setGrammarFile(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setGrammarFile(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void addClient(const ::Ice::Identity&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___addClient(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void start(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___start(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void stop(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___stop(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void shutdown(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___shutdown(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr getCurrentTime(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getCurrentTime(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::std::string getCWD(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getCWD(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::std::string getStatusString(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getStatusString(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::IdentitySeq getClients(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getClients(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class Client : virtual public ::Ice::Object
{
public:

    typedef ClientPrx ProxyType;
    typedef ClientPtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void onStart(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onStart(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void onStop(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onStop(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onAudioSourceChange(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void onEndOfStream(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onEndOfStream(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void onGrammarChange(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onGrammarChange(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onRecognitionResult(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void onUnregistrationFromServer(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___onUnregistrationFromServer(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

}

}

}

}

}

}

#endif
