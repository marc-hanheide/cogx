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

#include <asr-loquendo.h>
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
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__setAudioSource_name = "setAudioSource";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__setGrammarFile_name = "setGrammarFile";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__addClient_name = "addClient";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__start_name = "start";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__stop_name = "stop";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__shutdown_name = "shutdown";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCurrentTime_name = "getCurrentTime";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCWD_name = "getCWD";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getStatusString_name = "getStatusString";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getClients_name = "getClients";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onStart_name = "onStart";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onStop_name = "onStop";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onAudioSourceChange_name = "onAudioSourceChange";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onEndOfStream_name = "onEndOfStream";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onGrammarChange_name = "onGrammarChange";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onRecognitionResult_name = "onRecognitionResult";

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onUnregistrationFromServer_name = "onUnregistrationFromServer";

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::Server* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server* p) { return p; }

::Ice::Object* IceInternal::upCast(::de::dfki::lt::tr::dialogue::asr::loquendo::Client* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client* p) { return p; }

void
de::dfki::lt::tr::dialogue::asr::loquendo::time::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResultPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestListPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLatticePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapturePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFilePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::ClientPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client;
        v->__copyFrom(proxy);
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeWordHypothesisSeq(::IceInternal::BasicStream* __os, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr* begin, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(begin[i].get())));
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__readWordHypothesisSeq(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 4);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__WordHypothesisPtr, &v[i]);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeHypothesisSeq(::IceInternal::BasicStream* __os, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr* begin, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(begin[i].get())));
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__readHypothesisSeq(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 4);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__HypothesisPtr, &v[i]);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__write(::IceInternal::BasicStream* __os, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RejectionFlag v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 2);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RejectionFlag& v)
{
    ::Ice::Byte val;
    __is->read(val, 2);
    v = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RejectionFlag>(val);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeNodeSeq(::IceInternal::BasicStream* __os, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr* begin, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(begin[i].get())));
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__readNodeSeq(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodeSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 4);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__NodePtr, &v[i]);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeLinkSeq(::IceInternal::BasicStream* __os, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr* begin, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(begin[i].get())));
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::__readLinkSeq(::IceInternal::BasicStream* __is, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 4);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__LinkPtr, &v[i]);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::LoquendoException(const ::std::string& __ice_function, ::Ice::Int __ice_returnCode, const ::std::string& __ice_errorMessage) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    UserException(),
#else
    ::Ice::UserException(),
#endif
    function(__ice_function),
    returnCode(__ice_returnCode),
    errorMessage(__ice_errorMessage)
{
}

de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::~LoquendoException() throw()
{
}

static const char* __de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException_name = "de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException";

::std::string
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::ice_name() const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException_name;
}

::Ice::Exception*
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::ice_clone() const
{
    return new LoquendoException(*this);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::ice_throw() const
{
    throw *this;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException"), false);
    __os->startWriteSlice();
    __os->write(function);
    __os->write(returnCode);
    __os->write(errorMessage);
    __os->endWriteSlice();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->read(function);
    __is->read(returnCode);
    __is->read(errorMessage);
    __is->endReadSlice();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException was not generated with stream support";
    throw ex;
}

struct __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException;

const ::IceInternal::UserExceptionFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException", ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException");
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__LoquendoException__initializer() {} }
#endif

de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::ServerException(const ::std::string& __ice_message) :
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    UserException(),
#else
    ::Ice::UserException(),
#endif
    message(__ice_message)
{
}

de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::~ServerException() throw()
{
}

static const char* __de__dfki__lt__tr__dialogue__asr__loquendo__ServerException_name = "de::dfki::lt::tr::dialogue::asr::loquendo::ServerException";

::std::string
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::ice_name() const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__ServerException_name;
}

::Ice::Exception*
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::ice_clone() const
{
    return new ServerException(*this);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::ice_throw() const
{
    throw *this;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(::std::string("::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException"), false);
    __os->startWriteSlice();
    __os->write(message);
    __os->endWriteSlice();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->read(myId, false);
    }
    __is->startReadSlice();
    __is->read(message);
    __is->endReadSlice();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception de::dfki::lt::tr::dialogue::asr::loquendo::ServerException was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "exception de::dfki::lt::tr::dialogue::asr::loquendo::ServerException was not generated with stream support";
    throw ex;
}

struct __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException : public ::IceInternal::UserExceptionFactory
{
    virtual void
    createAndThrow()
    {
        throw ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException();
    }
};

static ::IceInternal::UserExceptionFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException;

const ::IceInternal::UserExceptionFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__Init()
    {
        ::IceInternal::factoryTable->addExceptionFactory("::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException", ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__Init()
    {
        ::IceInternal::factoryTable->removeExceptionFactory("::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException");
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__ServerException__initializer() {} }
#endif

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__newInstance() const
{
    return new TimeVal;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__newInstance() const
{
    return new RecognitionResult;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__newInstance() const
{
    return new NoRecognitionResult;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__newInstance() const
{
    return new WordHypothesis;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__newInstance() const
{
    return new Hypothesis;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__newInstance() const
{
    return new NBestList;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__newInstance() const
{
    return new Node;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__newInstance() const
{
    return new Link;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__newInstance() const
{
    return new WordLattice;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__newInstance() const
{
    return new AudioSource;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__newInstance() const
{
    return new PulseAudioPCMCapture;
}

const ::std::string&
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__newInstance() const
{
    return new RAWFile;
}

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__setAudioSource_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            __del->setAudioSource(as, __ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::setGrammarFile(const ::std::string& filename, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__setGrammarFile_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            __del->setGrammarFile(filename, __ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::addClient(const ::Ice::Identity& ident, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            __del->addClient(ident, __ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::start(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__start_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            __del->start(__ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::stop(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            __del->stop(__ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::shutdown(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            __del->shutdown(__ctx);
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

::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getCurrentTime(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCurrentTime_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            return __del->getCurrentTime(__ctx);
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

::std::string
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getCWD(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCWD_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            return __del->getCWD(__ctx);
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

::std::string
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getStatusString(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__getStatusString_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            return __del->getStatusString(__ctx);
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

::Ice::IdentitySeq
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getClients(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__de__dfki__lt__tr__dialogue__asr__loquendo__Server__getClients_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(__delBase.get());
            return __del->getClients(__ctx);
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
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Server::__newInstance() const
{
    return new Server;
}

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onStart(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onStart(__ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onStop(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onStop(__ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onAudioSourceChange(as, __ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onEndOfStream(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onEndOfStream(__ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onGrammarChange(const ::std::string& filename, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onGrammarChange(filename, __ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& res, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onRecognitionResult(res, __ctx);
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

void
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onUnregistrationFromServer(const ::std::string& reason, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client* __del = dynamic_cast< ::IceDelegate::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(__delBase.get());
            __del->onUnregistrationFromServer(reason, __ctx);
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
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_staticId()
{
    return ::de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client);
}

::IceProxy::Ice::Object*
IceProxy::de::dfki::lt::tr::dialogue::asr::loquendo::Client::__newInstance() const
{
    return new Client;
}

void
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__setAudioSource_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(as.get())));
        __os->writePendingObjects();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException&)
            {
                throw;
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

void
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::setGrammarFile(const ::std::string& filename, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__setGrammarFile_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(filename);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException&)
            {
                throw;
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

void
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::addClient(const ::Ice::Identity& ident, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__addClient_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ident.__write(__os);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::start(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__start_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException&)
            {
                throw;
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

void
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::stop(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__stop_name, ::Ice::Normal, __context);
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::shutdown(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__shutdown_name, ::Ice::Normal, __context);
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

::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getCurrentTime(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCurrentTime_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr __ret;
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::time::__patch__TimeValPtr, &__ret);
        __is->readPendingObjects();
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::std::string
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getCWD(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCWD_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    ::std::string __ret;
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::std::string
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getStatusString(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getStatusString_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    ::std::string __ret;
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::Ice::IdentitySeq
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getClients(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getClients_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    ::Ice::IdentitySeq __ret;
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        ::Ice::__readIdentitySeq(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onStart(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onStart_name, ::Ice::Normal, __context);
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onStop(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onStop_name, ::Ice::Normal, __context);
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onAudioSourceChange_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(as.get())));
        __os->writePendingObjects();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onEndOfStream(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onEndOfStream_name, ::Ice::Normal, __context);
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onGrammarChange(const ::std::string& filename, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onGrammarChange_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(filename);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& res, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onRecognitionResult_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(res.get())));
        __os->writePendingObjects();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onUnregistrationFromServer(const ::std::string& reason, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onUnregistrationFromServer_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(reason);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::setAudioSource(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_as(as)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->setAudioSource(_m_as, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& _m_as;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__setAudioSource_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(as, __current);
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
    catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException&)
    {
        throw;
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::setGrammarFile(const ::std::string& filename, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& filename, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_filename(filename)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->setGrammarFile(_m_filename, _current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
        const ::std::string& _m_filename;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__setGrammarFile_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(filename, __current);
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
    catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException&)
    {
        throw;
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::addClient(const ::Ice::Identity& ident, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Identity& ident, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_ident(ident)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->addClient(_m_ident, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::Ice::Identity& _m_ident;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__addClient_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(ident, __current);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::start(const ::Ice::Context* __context)
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
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            try
            {
                servant->start(_current);
                return ::Ice::DispatchOK;
            }
            catch(const ::Ice::UserException& __ex)
            {
                setUserException(__ex);
                return ::Ice::DispatchUserException;
            }
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__start_name, ::Ice::Normal, __context);
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
    catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException&)
    {
        throw;
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::stop(const ::Ice::Context* __context)
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
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->stop(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__stop_name, ::Ice::Normal, __context);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::shutdown(const ::Ice::Context* __context)
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
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->shutdown(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__shutdown_name, ::Ice::Normal, __context);
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

::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getCurrentTime(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getCurrentTime(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCurrentTime_name, ::Ice::Normal, __context);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr __result;
    try
    {
        _DirectI __direct(__result, __current);
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
    return __result;
}

::std::string
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getCWD(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::std::string& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getCWD(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::std::string& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getCWD_name, ::Ice::Normal, __context);
    ::std::string __result;
    try
    {
        _DirectI __direct(__result, __current);
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
    return __result;
}

::std::string
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getStatusString(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::std::string& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getStatusString(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::std::string& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getStatusString_name, ::Ice::Normal, __context);
    ::std::string __result;
    try
    {
        _DirectI __direct(__result, __current);
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
    return __result;
}

::Ice::IdentitySeq
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Server::getClients(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::IdentitySeq& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Server* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Server*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getClients(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::IdentitySeq& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Server__getClients_name, ::Ice::Normal, __context);
    ::Ice::IdentitySeq __result;
    try
    {
        _DirectI __direct(__result, __current);
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
    return __result;
}

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onStart(const ::Ice::Context* __context)
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
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onStart(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onStart_name, ::Ice::Normal, __context);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onStop(const ::Ice::Context* __context)
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
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onStop(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onStop_name, ::Ice::Normal, __context);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onAudioSourceChange(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& as, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_as(as)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onAudioSourceChange(_m_as, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr& _m_as;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onAudioSourceChange_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(as, __current);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onEndOfStream(const ::Ice::Context* __context)
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
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onEndOfStream(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onEndOfStream_name, ::Ice::Normal, __context);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onGrammarChange(const ::std::string& filename, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& filename, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_filename(filename)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onGrammarChange(_m_filename, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_filename;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onGrammarChange_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(filename, __current);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onRecognitionResult(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& res, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& res, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_res(res)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onRecognitionResult(_m_res, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr& _m_res;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onRecognitionResult_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(res, __current);
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

void
IceDelegateD::de::dfki::lt::tr::dialogue::asr::loquendo::Client::onUnregistrationFromServer(const ::std::string& reason, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::std::string& reason, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_reason(reason)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::de::dfki::lt::tr::dialogue::asr::loquendo::Client* servant = dynamic_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::Client*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->onUnregistrationFromServer(_m_reason, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        const ::std::string& _m_reason;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __de__dfki__lt__tr__dialogue__asr__loquendo__Client__onUnregistrationFromServer_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(reason, __current);
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

de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::TimeVal(::Ice::Long __ice_sec, ::Ice::Int __ice_usec) :
    sec(__ice_sec),
    usec(__ice_usec)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(sec);
    __os->write(usec);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(sec);
    __is->read(usec);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__time__TimeVal__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::time::__patch__TimeValPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::time::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::time::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeVal& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__write(::IceInternal::BasicStream* __os) const
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__read(::IceInternal::BasicStream* __is, bool __rid)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__RecognitionResult__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__RecognitionResultPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResultPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids[3] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids + 3, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids[3]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    RecognitionResult::__write(__os);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    RecognitionResult::__read(__is, true);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NoRecognitionResult__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__NoRecognitionResultPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResultPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResultPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResultPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NoRecognitionResult& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::WordHypothesis(const ::std::string& __ice_word, const ::std::string& __ice_lang, ::Ice::Float __ice_acousticScore, ::Ice::Float __ice_confidence, ::Ice::Int __ice_startFrame, ::Ice::Int __ice_endFrame) :
    word(__ice_word),
    lang(__ice_lang),
    acousticScore(__ice_acousticScore),
    confidence(__ice_confidence),
    startFrame(__ice_startFrame),
    endFrame(__ice_endFrame)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(word);
    __os->write(lang);
    __os->write(acousticScore);
    __os->write(confidence);
    __os->write(startFrame);
    __os->write(endFrame);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(word);
    __is->read(lang);
    __is->read(acousticScore);
    __is->read(confidence);
    __is->read(startFrame);
    __is->read(endFrame);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordHypothesis__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__WordHypothesisPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesis& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::Hypothesis(const ::std::string& __ice_str, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisSeq& __ice_words, ::Ice::Float __ice_acousticScore, ::Ice::Float __ice_confidence, const ::std::string& __ice_roName, const ::std::string& __ice_roRule) :
    str(__ice_str),
    words(__ice_words),
    acousticScore(__ice_acousticScore),
    confidence(__ice_confidence),
    roName(__ice_roName),
    roRule(__ice_roRule)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__incRef()
{
    __gcIncRef();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__decRef()
{
    __gcDecRef();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__addObject(::IceInternal::GCCountMap& _c)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__usesClasses()
{
    return true;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__gcReachable(::IceInternal::GCCountMap& _c) const
{
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisSeq::const_iterator _i0 = words.begin(); _i0 != words.end(); ++_i0)
        {
            if((*_i0))
            {
                ::IceInternal::upCast((*_i0).get())->__addObject(_c);
            }
        }
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__gcClear()
{
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordHypothesisSeq::iterator _i0 = words.begin(); _i0 != words.end(); ++_i0)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(str);
    if(words.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeWordHypothesisSeq(__os, &words[0], &words[0] + words.size());
    }
    __os->write(acousticScore);
    __os->write(confidence);
    __os->write(roName);
    __os->write(roRule);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(str);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__readWordHypothesisSeq(__is, words);
    __is->read(acousticScore);
    __is->read(confidence);
    __is->read(roName);
    __is->read(roRule);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Hypothesis__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__HypothesisPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Hypothesis& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::NBestList(const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr& __ice_timeAnchor, ::Ice::Float __ice_snr, ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RejectionFlag __ice_rejectionAdvice, const ::std::string& __ice_roName, const ::std::string& __ice_roRule, ::Ice::Int __ice_speechStartFrame, ::Ice::Int __ice_speechEndFrame, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisSeq& __ice_hypos) :
    timeAnchor(__ice_timeAnchor),
    snr(__ice_snr),
    rejectionAdvice(__ice_rejectionAdvice),
    roName(__ice_roName),
    roRule(__ice_roRule),
    speechStartFrame(__ice_speechStartFrame),
    speechEndFrame(__ice_speechEndFrame),
    hypos(__ice_hypos)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestListPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids[3] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids + 3, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids[3]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__incRef()
{
    __gcIncRef();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__decRef()
{
    __gcDecRef();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__addObject(::IceInternal::GCCountMap& _c)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__usesClasses()
{
    return true;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__gcReachable(::IceInternal::GCCountMap& _c) const
{
    if(timeAnchor)
    {
        ::IceInternal::upCast(timeAnchor.get())->__addObject(_c);
    }
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisSeq::const_iterator _i0 = hypos.begin(); _i0 != hypos.end(); ++_i0)
        {
            if((*_i0))
            {
                ::IceInternal::upCast((*_i0).get())->__addObject(_c);
            }
        }
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__gcClear()
{
    if(timeAnchor)
    {
        if(::IceInternal::upCast(timeAnchor.get())->__usesClasses())
        {
            ::IceInternal::upCast(timeAnchor.get())->__decRefUnsafe();
            timeAnchor.__clearHandleUnsafe();
        }
        else
        {
            timeAnchor = 0;
        }
    }
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::HypothesisSeq::iterator _i0 = hypos.begin(); _i0 != hypos.end(); ++_i0)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(::Ice::ObjectPtr(::IceInternal::upCast(timeAnchor.get())));
    __os->write(snr);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__write(__os, rejectionAdvice);
    __os->write(roName);
    __os->write(roRule);
    __os->write(speechStartFrame);
    __os->write(speechEndFrame);
    if(hypos.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeHypothesisSeq(__os, &hypos[0], &hypos[0] + hypos.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    RecognitionResult::__write(__os);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::time::__patch__TimeValPtr, &timeAnchor);
    __is->read(snr);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__read(__is, rejectionAdvice);
    __is->read(roName);
    __is->read(roRule);
    __is->read(speechStartFrame);
    __is->read(speechEndFrame);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__readHypothesisSeq(__is, hypos);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    RecognitionResult::__read(__is, true);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__NBestList__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__NBestListPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestListPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestListPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestListPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NBestList& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::Node(::Ice::Int __ice_id, ::Ice::Float __ice_time, ::Ice::Float __ice_frame) :
    id(__ice_id),
    time(__ice_time),
    frame(__ice_frame)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(id);
    __os->write(time);
    __os->write(frame);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(id);
    __is->read(time);
    __is->read(frame);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::Node was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::Node was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Node__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__NodePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Node& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::Link(::Ice::Int __ice_id, ::Ice::Int __ice_start, ::Ice::Int __ice_end, const ::std::string& __ice_word, ::Ice::Float __ice_confidence, ::Ice::Float __ice_searchScore, ::Ice::Float __ice_sequenceScore, ::Ice::Float __ice_acoustic) :
    id(__ice_id),
    start(__ice_start),
    end(__ice_end),
    word(__ice_word),
    confidence(__ice_confidence),
    searchScore(__ice_searchScore),
    sequenceScore(__ice_sequenceScore),
    acoustic(__ice_acoustic)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(id);
    __os->write(start);
    __os->write(end);
    __os->write(word);
    __os->write(confidence);
    __os->write(searchScore);
    __os->write(sequenceScore);
    __os->write(acoustic);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(id);
    __is->read(start);
    __is->read(end);
    __is->read(word);
    __is->read(confidence);
    __is->read(searchScore);
    __is->read(sequenceScore);
    __is->read(acoustic);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::Link was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::Link was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__Link__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__LinkPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::Link& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::WordLattice(const ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr& __ice_timeAnchor, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodeSeq& __ice_nodes, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkSeq& __ice_links) :
    timeAnchor(__ice_timeAnchor),
    nodes(__ice_nodes),
    links(__ice_links)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLatticePtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids[3] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids + 3, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids[3]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids[2];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_ids[2];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__incRef()
{
    __gcIncRef();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__decRef()
{
    __gcDecRef();
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__addObject(::IceInternal::GCCountMap& _c)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__usesClasses()
{
    return true;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__gcReachable(::IceInternal::GCCountMap& _c) const
{
    if(timeAnchor)
    {
        ::IceInternal::upCast(timeAnchor.get())->__addObject(_c);
    }
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodeSeq::const_iterator _i0 = nodes.begin(); _i0 != nodes.end(); ++_i0)
        {
            if((*_i0))
            {
                ::IceInternal::upCast((*_i0).get())->__addObject(_c);
            }
        }
    }
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkSeq::const_iterator _i0 = links.begin(); _i0 != links.end(); ++_i0)
        {
            if((*_i0))
            {
                ::IceInternal::upCast((*_i0).get())->__addObject(_c);
            }
        }
    }
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__gcClear()
{
    if(timeAnchor)
    {
        if(::IceInternal::upCast(timeAnchor.get())->__usesClasses())
        {
            ::IceInternal::upCast(timeAnchor.get())->__decRefUnsafe();
            timeAnchor.__clearHandleUnsafe();
        }
        else
        {
            timeAnchor = 0;
        }
    }
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::NodeSeq::iterator _i0 = nodes.begin(); _i0 != nodes.end(); ++_i0)
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
    {
        for(::de::dfki::lt::tr::dialogue::asr::loquendo::result::LinkSeq::iterator _i0 = links.begin(); _i0 != links.end(); ++_i0)
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
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(::Ice::ObjectPtr(::IceInternal::upCast(timeAnchor.get())));
    if(nodes.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeNodeSeq(__os, &nodes[0], &nodes[0] + nodes.size());
    }
    if(links.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__writeLinkSeq(__os, &links[0], &links[0] + links.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    RecognitionResult::__write(__os);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::time::__patch__TimeValPtr, &timeAnchor);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__readNodeSeq(__is, nodes);
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::__readLinkSeq(__is, links);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    RecognitionResult::__read(__is, true);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResult::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__result__WordLattice__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__WordLatticePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLatticePtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLatticePtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLatticePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::result::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::result::WordLattice& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_ids[1];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__write(::IceInternal::BasicStream* __os) const
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
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__read(::IceInternal::BasicStream* __is, bool __rid)
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
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__AudioSource__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::__patch__AudioSourcePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapturePtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids[3] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids + 3, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids[3]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids[2];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_ids[2];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    AudioSource::__write(__os);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    AudioSource::__read(__is, true);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__PulseAudioPCMCapture__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::__patch__PulseAudioPCMCapturePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapturePtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapturePtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapturePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::PulseAudioPCMCapture& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::RAWFile(const ::std::string& __ice_path) :
    path(__ice_path)
{
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_clone() const
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFilePtr __p = new ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile(*this);
    return __p;
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids[3] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids + 3, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids[3]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids[2];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_ids[2];
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(path);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    AudioSource::__write(__os);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__write(__os);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(path);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    AudioSource::__read(__is, true);
#else
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSource::__read(__is, true);
#endif
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile was not generated with stream support";
    throw ex;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId());
        return new ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_Ptr = new __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile;

const ::Ice::ObjectFactoryPtr&
de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_factory()
{
    return __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile_Ptr;
}

class __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile__Init
{
public:

    __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId(), ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_factory());
    }

    ~__F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId());
    }
};

static __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile__Init __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile__i;

#ifdef __APPLE__
extern "C" { void __F__de__dfki__lt__tr__dialogue__asr__loquendo__RAWFile__initializer() {} }
#endif

void 
de::dfki::lt::tr::dialogue::asr::loquendo::__patch__RAWFilePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFilePtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFilePtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFilePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::RAWFile& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::Server"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__Server_ids[1];
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___setAudioSource(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr as;
    __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::__patch__AudioSourcePtr, &as);
    __is->readPendingObjects();
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        setAudioSource(as, __current);
    }
    catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___setGrammarFile(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string filename;
    __is->read(filename);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        setGrammarFile(filename, __current);
    }
    catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___addClient(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Ice::Identity ident;
    ident.__read(__is);
    __is->endReadEncaps();
    addClient(ident, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___start(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    try
    {
        start(__current);
    }
    catch(const ::de::dfki::lt::tr::dialogue::asr::loquendo::LoquendoException& __ex)
    {
        __os->write(__ex);
        return ::Ice::DispatchUserException;
    }
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___stop(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    stop(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___shutdown(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    shutdown(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___getCurrentTime(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::de::dfki::lt::tr::dialogue::asr::loquendo::time::TimeValPtr __ret = getCurrentTime(__current);
    __os->write(::Ice::ObjectPtr(::IceInternal::upCast(__ret.get())));
    __os->writePendingObjects();
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___getCWD(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::std::string __ret = getCWD(__current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___getStatusString(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::std::string __ret = getStatusString(__current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::___getClients(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::IdentitySeq __ret = getClients(__current);
    if(__ret.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::Ice::__writeIdentitySeq(__os, &__ret[0], &__ret[0] + __ret.size());
    }
    return ::Ice::DispatchOK;
}

static ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Server_all[] =
{
    "addClient",
    "getCWD",
    "getClients",
    "getCurrentTime",
    "getStatusString",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "setAudioSource",
    "setGrammarFile",
    "shutdown",
    "start",
    "stop"
};

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Server::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__de__dfki__lt__tr__dialogue__asr__loquendo__Server_all, __de__dfki__lt__tr__dialogue__asr__loquendo__Server_all + 14, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __de__dfki__lt__tr__dialogue__asr__loquendo__Server_all)
    {
        case 0:
        {
            return ___addClient(in, current);
        }
        case 1:
        {
            return ___getCWD(in, current);
        }
        case 2:
        {
            return ___getClients(in, current);
        }
        case 3:
        {
            return ___getCurrentTime(in, current);
        }
        case 4:
        {
            return ___getStatusString(in, current);
        }
        case 5:
        {
            return ___ice_id(in, current);
        }
        case 6:
        {
            return ___ice_ids(in, current);
        }
        case 7:
        {
            return ___ice_isA(in, current);
        }
        case 8:
        {
            return ___ice_ping(in, current);
        }
        case 9:
        {
            return ___setAudioSource(in, current);
        }
        case 10:
        {
            return ___setGrammarFile(in, current);
        }
        case 11:
        {
            return ___shutdown(in, current);
        }
        case 12:
        {
            return ___start(in, current);
        }
        case 13:
        {
            return ___stop(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::Server::__write(::IceInternal::BasicStream* __os) const
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
de::dfki::lt::tr::dialogue::asr::loquendo::Server::__read(::IceInternal::BasicStream* __is, bool __rid)
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
de::dfki::lt::tr::dialogue::asr::loquendo::Server::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::Server was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::Server::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::Server was not generated with stream support";
    throw ex;
}

void 
de::dfki::lt::tr::dialogue::asr::loquendo::__patch__ServerPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::ServerPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::Server::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::Server& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::Server& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::Server& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::Server& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids[2] =
{
    "::Ice::Object",
    "::de::dfki::lt::tr::dialogue::asr::loquendo::Client"
};

bool
de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids, __de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids + 2, _s);
}

::std::vector< ::std::string>
de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids[0], &__de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids[2]);
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_id(const ::Ice::Current&) const
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids[1];
}

const ::std::string&
de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_staticId()
{
    return __de__dfki__lt__tr__dialogue__asr__loquendo__Client_ids[1];
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onStart(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    onStart(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onStop(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    onStop(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onAudioSourceChange(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::de::dfki::lt::tr::dialogue::asr::loquendo::AudioSourcePtr as;
    __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::__patch__AudioSourcePtr, &as);
    __is->readPendingObjects();
    __is->endReadEncaps();
    onAudioSourceChange(as, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onEndOfStream(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    onEndOfStream(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onGrammarChange(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string filename;
    __is->read(filename);
    __is->endReadEncaps();
    onGrammarChange(filename, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onRecognitionResult(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::de::dfki::lt::tr::dialogue::asr::loquendo::result::RecognitionResultPtr res;
    __is->read(::de::dfki::lt::tr::dialogue::asr::loquendo::result::__patch__RecognitionResultPtr, &res);
    __is->readPendingObjects();
    __is->endReadEncaps();
    onRecognitionResult(res, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::___onUnregistrationFromServer(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::std::string reason;
    __is->read(reason);
    __is->endReadEncaps();
    onUnregistrationFromServer(reason, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __de__dfki__lt__tr__dialogue__asr__loquendo__Client_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "onAudioSourceChange",
    "onEndOfStream",
    "onGrammarChange",
    "onRecognitionResult",
    "onStart",
    "onStop",
    "onUnregistrationFromServer"
};

::Ice::DispatchStatus
de::dfki::lt::tr::dialogue::asr::loquendo::Client::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__de__dfki__lt__tr__dialogue__asr__loquendo__Client_all, __de__dfki__lt__tr__dialogue__asr__loquendo__Client_all + 11, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __de__dfki__lt__tr__dialogue__asr__loquendo__Client_all)
    {
        case 0:
        {
            return ___ice_id(in, current);
        }
        case 1:
        {
            return ___ice_ids(in, current);
        }
        case 2:
        {
            return ___ice_isA(in, current);
        }
        case 3:
        {
            return ___ice_ping(in, current);
        }
        case 4:
        {
            return ___onAudioSourceChange(in, current);
        }
        case 5:
        {
            return ___onEndOfStream(in, current);
        }
        case 6:
        {
            return ___onGrammarChange(in, current);
        }
        case 7:
        {
            return ___onRecognitionResult(in, current);
        }
        case 8:
        {
            return ___onStart(in, current);
        }
        case 9:
        {
            return ___onStop(in, current);
        }
        case 10:
        {
            return ___onUnregistrationFromServer(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::Client::__write(::IceInternal::BasicStream* __os) const
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
de::dfki::lt::tr::dialogue::asr::loquendo::Client::__read(::IceInternal::BasicStream* __is, bool __rid)
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
de::dfki::lt::tr::dialogue::asr::loquendo::Client::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::Client was not generated with stream support";
    throw ex;
}

void
de::dfki::lt::tr::dialogue::asr::loquendo::Client::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type de::dfki::lt::tr::dialogue::asr::loquendo::Client was not generated with stream support";
    throw ex;
}

void 
de::dfki::lt::tr::dialogue::asr::loquendo::__patch__ClientPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::de::dfki::lt::tr::dialogue::asr::loquendo::ClientPtr* p = static_cast< ::de::dfki::lt::tr::dialogue::asr::loquendo::ClientPtr*>(__addr);
    assert(p);
    *p = ::de::dfki::lt::tr::dialogue::asr::loquendo::ClientPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::de::dfki::lt::tr::dialogue::asr::loquendo::Client::ice_staticId(), v->ice_id());
    }
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator==(const ::de::dfki::lt::tr::dialogue::asr::loquendo::Client& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::Client& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
de::dfki::lt::tr::dialogue::asr::loquendo::operator<(const ::de::dfki::lt::tr::dialogue::asr::loquendo::Client& l, const ::de::dfki::lt::tr::dialogue::asr::loquendo::Client& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
