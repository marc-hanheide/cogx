// ----------------------------------------------------------------------------
// Copyright (C) 2010-2011 DFKI GmbH Talking Robots 
// Miroslav Janicek (miroslav.janicek@dfki.de) 
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// ----------------------------------------------------------------------------

extern "C" {
#include <dlfcn.h>
#include <semaphore.h>
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
}
#include "LoquendoRecogniserServer.h"

#include "Version.h"

#include "PCMCapture.h"
#include "PCMCaptureAudioSource.h"
#include "RAWFileAudioSource.h"

#include "SLFReader.h"

#include <log4cxx/logger.h>

using namespace std;
using namespace log4cxx;
using namespace LoqASR;
using namespace LoqASR::result;

LoggerPtr serverLogger(Logger::getLogger("loquendo-asr.server"));

#include <vector>
#include <sstream>
#include <iomanip>

static time::TimeValPtr latestTime = 0;

time::TimeValPtr
getCurrentTimeOfDay()
{
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);

	return new time::TimeVal(tv.tv_sec, tv.tv_usec);
}

LoquendoException
LoquendoRecogniserServer::newLoquendoException(const char * functionName, int returnCode)
{
	char * str;
	lasrxGetErrorMessage(instance->getHandle(), &str);
	string message(str);
	lasrxFree(str);
	return LoquendoException(functionName, returnCode, message);
}

// event callback function
static lasrxResultType
__GetEvent(lasrxHandleType hInstance, lasrxPointerType pUser, lasrxIntType nEvent, lasrxIntType nReason, lasrxStringType szId, lasrxPointerType pEventData, lasrxIntType nEventDataSize)
{
    lasrxStringType szInstanceId;
    lasrxGetInstanceId(hInstance, &szInstanceId);
	string instId(szInstanceId);
	lasrxFree(szInstanceId);

	string name = "";
    switch (nEvent) {
    case LASRX_EVENT_START_VOICE_DETECTED:
		name = "LASRX_EVENT_START_VOICE_DETECTED";
		latestTime = getCurrentTimeOfDay();
        break;
    case LASRX_EVENT_END_VOICE_DETECTED:
        name = "LASRX_EVENT_END_VOICE_DETECTED";
        break;
    case LASRX_EVENT_PROMPT_STOP:
        name = "LASRX_EVENT_PROMPT_STOP";
        break;
    case LASRX_EVENT_RESTART_FOR_OOV:
        name = "LASRX_EVENT_RESTART_FOR_OOV";
        break;
    case LASRX_EVENT_RESTART_FOR_BGN:
        name = "LASRX_EVENT_RESTART_FOR_BGN";
        break;
    case LASRX_EVENT_RESTART_FOR_EPD_ERROR:
        name = "LASRX_EVENT_RESTART_FOR_EPD_ERROR";
        break;
    case LASRX_EVENT_SWITCHED_MODE:
        name = "LASRX_EVENT_SWITCHED_MODE";
        break;
    case LASRX_EVENT_END_RECOG:
        name = "LASRX_EVENT_END_RECOG";
        break;
    case LASRX_EVENT_END_TRECOG:
        name = "LASRX_EVENT_END_TRECOG";
        break;
    case LASRX_EVENT_END_GENERATE_RO:
        name = "LASRX_EVENT_END_GENERATE_RO";
        break;
    case LASRX_EVENT_END_VERIFY:
        name = "LASRX_EVENT_END_VERIFY";
        break;
    case LASRX_EVENT_END_MVERIFY:
        name = "LASRX_EVENT_END_MVERIFY";
        break;
    case LASRX_EVENT_END_ENROLL:
        name = "LASRX_EVENT_END_ENROLL";
        break;
    case LASRX_EVENT_END_SEGMENT:
        name = "LASRX_EVENT_END_SEGMENT";
        break;
    case LASRX_EVENT_END_RECORD:
        name = "LASRX_EVENT_END_RECORD";
        break;
    default:
        name = "UNKNOWN";
        break;
    }

	if (!name.empty()) {
		LOG4CXX_TRACE(serverLogger, "got event: " << name << " (reason = " << nReason << ")");
	}

//    if (nReason != LASRX_RETCODE_OK) __ReportLasrxError("__GetEvent", hInstance, szId, nReason);
    return LASRX_RETCODE_OK;
}

// audio event callback function
static lasrxResultType
LASRX_STDCALL
__GetAudioEvent(lasrxHandleType hInstance, lasrxPointerType pUser, lasrxIntType nEvent)
{
    lasrxStringType szInstanceId;
    lasrxGetInstanceId(hInstance, &szInstanceId);
	string instId(szInstanceId);
	lasrxFree(szInstanceId);

	string name = "";
    switch (nEvent) {
    case LASRX_EVENT_AUDIO_WAITING_SAMPLES:
		break;
    case LASRX_EVENT_AUDIO_RUNNING:
        break;
    case LASRX_EVENT_AUDIO_STOPPED:
        name = "LASRX_EVENT_AUDIO_STOPPED";
        break;
    case LASRX_EVENT_AUDIO_WAITING_STOPPED:
        name = "LASRX_EVENT_AUDIO_WAITING_STOPPED";
        break;
    default:
        name = "UNKNOWN";
        break;
    }

	if (!name.empty()) {
		LOG4CXX_TRACE(serverLogger, "got audio event: " << name);
	}

    return LASRX_RETCODE_OK;
}

//------------------------------------------------------------------------------

void
LoquendoRecogniserServer::loadPCMCaptureAudioSource()
{
	LOG4CXX_TRACE(serverLogger, "preparing audio source");
	LOG4CXX_TRACE(serverLogger, "expected sample frequency: " << instance->getExpectedSamplingFrequency() << " Hz");
	LOG4CXX_TRACE(serverLogger, "expected frame rate: " << instance->getExpectedFrameRate());  // XXX of what?

	pa_sample_format_t paFormat;
	switch (session->getAudioMode()) {
		case LASRX_SAMPLES_AL:
			paFormat = PA_SAMPLE_ALAW;
			break;
		case LASRX_SAMPLES_UL:
			paFormat = PA_SAMPLE_ULAW;
			break;
		case LASRX_SAMPLES_LIN16:
			paFormat = PA_SAMPLE_S16LE;
			break;
		default:
			throw ServerException("unknown audio mode");
		}

	audioSource = new PCMCaptureAudioSource(Logger::getLogger(serverLogger->getParent()->getName() + ".audiosource"), "Loquendo ASR server", paFormat, instance->getExpectedSamplingFrequency());
	audioSource->registerSource(instance->getHandle());
}

void
LoquendoRecogniserServer::loadRAWFileAudioSource(const string & path)
{
	try {
		audioSource = new RAWFileAudioSource(Logger::getLogger(serverLogger->getParent()->getName() + ".audiosource"), path);
		audioSource->registerSource(instance->getHandle());
	}
	catch (string err) {
		LOG4CXX_ERROR(serverLogger, err);
	}
}

//------------------------------------------------------------------------------

int
LoquendoRecogniserServer::buildRO(const std::string & grammarFile, const std::string & sourceName)
{
	LOG4CXX_DEBUG(serverLogger, "building ROs");

	lasrxResultType nRetCode = LASRX_RETCODE_OK;
//	lasrxStringType szRPName = (char *) LASRX_RPNAME_DEFAULT;

	instance->otfSetGrammarROGenerationDirectives(grammarFile, sourceName);
	string roId = instance->otfGenerateRO();
	LOG4CXX_DEBUG(serverLogger, "generate started: ID = " << roId);

	LOG4CXX_INFO(serverLogger, "ROs ready");
	return nRetCode;
}

NBestListPtr
LoquendoRecogniserServer::getNBestList(int maxHypos)
{
	NBestListPtr nBest = new NBestList();
	nBest->hypos = vector<HypothesisPtr>();

	pair<string,string> rROnr = instance->getRRRONameAndRule();
	nBest->roName = rROnr.first;
	nBest->roRule = rROnr.second;

	pair<int,int> limits = instance->getRRSpeechLimits();
	nBest->speechStartFrame = limits.first;
	nBest->speechEndFrame = limits.second;

	switch (instance->getRRRejectionAdvice()) {
	case LASRX_FALSE:
		nBest->rejectionAdvice = RecognitionFalse;
		break;
	case LASRX_REJECTION_NOMATCH:
		nBest->rejectionAdvice = RecognitionNomatch;
		break;
	default:
		throw ServerException("Unknown rejection advice");
	}

	nBest->snr = instance->getRRSignalToNoiseRatio();

	int numHypos = instance->getRRNumberOfHypothesis();
    for (int i = 0; i < numHypos && i < maxHypos; i++) {

		HypothesisPtr hypo = new Hypothesis();
		hypo->words = vector<WordHypothesisPtr>();

		pair<string,string> hROnr = instance->getRRHypothesisRONameAndRule(i);
		hypo->roName = hROnr.first;
		hypo->roRule = hROnr.second;

		hypo->acousticScore = instance->getRRHypothesisAcousticScore(i);
		hypo->confidence = instance->getRRHypothesisConfidence(i);
		hypo->str = instance->getRRHypothesisString(i);

		int numWords = instance->getRRHypothesisNumberOfWords(i);
        for (int j = 0; j < numWords; j++) {

			WordHypothesisPtr whypo = new WordHypothesis();

			whypo->word = instance->getRRWordHypothesisString(i, j);
			whypo->acousticScore = instance->getRRWordHypothesisAcousticScore(i, j);
			whypo->confidence = instance->getRRWordHypothesisConfidence(i, j);

			pair<int,int> segm = instance->getRRWordHypothesisSegmentation(i, j);
			whypo->startFrame = segm.first;
			whypo->endFrame = segm.second;

			whypo->lang = instance->getRRWordHypothesisLanguage(i, j);

			hypo->words.push_back(whypo);
        }
		nBest->hypos.push_back(hypo);
	}

	nBest->timeAnchor = latestTime;
	return nBest;
}

void
LoquendoRecogniserServer::setupLatticeExport()
{
	if (exportWordLattices) {
		instance->setInstanceParam("com.loquendo.asr.lattice.filename", latticeFilename);
		instance->setInstanceParam("com.loquendo.asr.lattice.confidencereject", "3");
		instance->setInstanceParam("com.loquendo.asr.lattice.confidencerejectextern", "1");
	}

}

WordLatticePtr
LoquendoRecogniserServer::getWordLattice()
{
	SLFReader reader;
	WordLatticePtr wl = reader.readLatticeFromFile(latticeFilename);
	if (wl) {
		LOG4CXX_DEBUG(serverLogger, "removing the lattice file `" << latticeFilename << "'");
		unlink(latticeFilename.c_str());
		setupLatticeExport();
	}
	wl->timeAnchor = latestTime;
	return wl;
}

int
LoquendoRecogniserServer::recog(const std::string & roName, const char * roRule)
{
	LOG4CXX_INFO(serverLogger, "recog(\"" << roName << "\", \"" << roRule << "\")");
	stringstream nss(stringstream::in|stringstream::out);
	nss << (const char *) LASRX_RPNAME_DEFAULT << "/" << roName;

	recognitionRunning = true;

	if (recognitionThread == NULL || !recognitionThread->isRunning()) {
		if (recognitionThread) {
			LOG4CXX_DEBUG(serverLogger, "killing an inactive recognition thread");
			delete recognitionThread;
		}
		try {
			instance->clearROs();
			instance->addRO(nss.str(), roRule);
			LOG4CXX_DEBUG(serverLogger, "no recognition thread active, will start a new one");
			recognitionThread = new RecognitionThread(Logger::getLogger(serverLogger->getParent()->getName() + ".recog-thread"), *this);
		}
		catch (const InstanceException & e) {
			LOG4CXX_ERROR(serverLogger, "instance exception: " << e.function << " (" << e.returnCode << "): " << e.message);
		}
	}
	else {
		LOG4CXX_DEBUG(serverLogger, "recognition thread already running");
	}
	LOG4CXX_TRACE(serverLogger, "recog(...) done");
	return LASRX_RETCODE_OK;
}

void
LoquendoRecogniserServer::throwInstanceException(const InstanceException & e)
{
	LOG4CXX_DEBUG(serverLogger, "throwing an instance exception");
	throw e;
}

//------------------------------------------------------------------------------

LoquendoRecogniserServer::LoquendoRecogniserServer(const Ice::CommunicatorPtr & ic_, const std::string & sessionFile_, const std::string & grammarFile_, bool doAudiodumps_, const std::string & audiodumpPrefix_, const std::string & audiodumpSuffix_, bool exportWordLattices_)
: session(NULL), instance(NULL), audioSource(NULL),
		sessionFile(sessionFile_), grammarFile(grammarFile_),
		ic(ic_),
		doAudiodumps(doAudiodumps_), audiodumpPrefix(audiodumpPrefix_), audiodumpSuffix(audiodumpSuffix_),
		exportWordLattices(exportWordLattices_), latticeFilename("/tmp/latest.slf"),
		recognitionRunning(false), recognitionThread(0)
{
	LOG4CXX_INFO(serverLogger, "starting up the ASR server");

	if (doAudiodumps) {
		LOG4CXX_INFO(serverLogger, "will dump audio files; prefix=\"" << audiodumpPrefix << "\", "
				<<  "suffix=\"" << audiodumpSuffix << "\"");
	}

	session = new LoquendoSession(sessionFile.c_str());

	LOG4CXX_INFO(serverLogger, "initialised " << session->getSystemVersionString());
	LOG4CXX_INFO(serverLogger, "system info: " << session->getSystemInfo());

	session->setAudioMode(LASRX_SAMPLES_AL);

	instance = session->newInstance();
	instance->setCallbackGetEvent(__GetEvent);
	instance->setAudioGetEvent(__GetAudioEvent);

	setupLatticeExport();

	if (buildRO(grammarFile.c_str(), "dynamic")) {
		throw ServerException("unable to build recognition object");
	}

	setAudioSource(new LoqASR::PulseAudioPCMCapture());

	pthread_mutex_init(&recogMutex, NULL);

	LOG4CXX_INFO(serverLogger, "ready");
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

LoquendoRecogniserServer::~LoquendoRecogniserServer()
{
	LOG4CXX_INFO(serverLogger, "shutting down the server");
	stop();

	if (instance) {
		delete instance;
	}
	if (session) {
		delete session;
	}
	if (audioSource) {
		delete audioSource;
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
LoquendoRecogniserServer::setAudioSource(const LoqASR::AudioSourcePtr & as, const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received setAudioSource(...)");
	setAudioSource(as);
}


void
LoquendoRecogniserServer::setAudioSource(const LoqASR::AudioSourcePtr & as)
{
	bool wasRunning = recognitionRunning;
	if (instance->getState() != LASRX_STATE_IDLE) {
		wasRunning = recognitionRunning;
		recognitionRunning = false;
		instance->stop();
	}

	pthread_mutex_lock(&recogMutex);

	if (audioSource) {
//		audioSource->unregisterSource();
		delete audioSource;
		audioSource = NULL;
	}

	if (PulseAudioPCMCapturePtr as_pcm = PulseAudioPCMCapturePtr::dynamicCast(as)) {
		LOG4CXX_INFO(serverLogger, "setting audio source to PCM capture");
		loadPCMCaptureAudioSource();
	}
	else if (RAWFilePtr as_raw = RAWFilePtr::dynamicCast(as)) {
		LOG4CXX_INFO(serverLogger, "setting audio source to RAW file: \"" << as_raw->path << "\"");
		loadRAWFileAudioSource(as_raw->path);
	}

	if (audioSource) {
		// we had success
		for (vector<ClientPrx>::iterator it = listeners.begin(); it != listeners.end(); it++) {
			Ice::Identity ident = (*it)->ice_getIdentity();
			LOG4CXX_DEBUG(serverLogger, "notifying listener " << ident.name << " on audio source change"); 
			try {
				(*it)->onAudioSourceChange(as);
			}
			catch (...) {
				LOG4CXX_WARN(serverLogger, "failed to tell listener " << ident.name << " about the grammar file change");
			}
		}
	}

	pthread_mutex_unlock(&recogMutex);

	if (wasRunning) {
		LOG4CXX_TRACE(serverLogger, "starting again");
		start();
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
LoquendoRecogniserServer::setGrammarFile(const string & filename, const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received setGrammarFile(\"" << filename << "\")");
	grammarFile = filename;

	bool wasRunning = recognitionRunning;
	if (instance->getState() != LASRX_STATE_IDLE) {
		instance->stop();
		wasRunning = recognitionRunning;
		recognitionRunning = false;
	}

	pthread_mutex_lock(&recogMutex);

	instance->otfDeleteRO("$$default/dynamic");
	if (buildRO(grammarFile.c_str(), "dynamic")) {
		throw new ServerException("unable to build recognition object");
	}
	else {
		for (vector<ClientPrx>::iterator it = listeners.begin(); it != listeners.end(); it++) {
			Ice::Identity ident = (*it)->ice_getIdentity();
			LOG4CXX_DEBUG(serverLogger, "notifying listener " << ident.name << " on grammar file change"); 
			try {
				(*it)->onGrammarChange(filename);
			}
			catch (...) {
				LOG4CXX_WARN(serverLogger, "failed to tell listener " << ident.name << " about the grammar file change");
			}
		}
	}
	pthread_mutex_unlock(&recogMutex);

	if (wasRunning) {
		LOG4CXX_TRACE(serverLogger, "starting again");
		start();
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
LoquendoRecogniserServer::start(const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received start()");
	start();
}

void
LoquendoRecogniserServer::start()
{
	if (!recognitionRunning) {
		LOG4CXX_INFO(serverLogger, "starting");
		if (audioSource) {
			audioSource->start();
			recog("dynamic", NULL);

			for (vector<ClientPrx>::iterator it = listeners.begin(); it != listeners.end(); it++) {
				Ice::Identity ident = (*it)->ice_getIdentity();
				LOG4CXX_DEBUG(serverLogger, "notifying listener " << ident.name << " on starting"); 
				try {
					(*it)->onStart();
				}
				catch (...) {
					LOG4CXX_WARN(serverLogger, "failed to tell listener " << ident.name << " about the start");
				}
			}
		}
		else {
			LOG4CXX_ERROR(serverLogger, "audiosource is NULL, not doing anything");
		}
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
LoquendoRecogniserServer::stop(const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received stop()");
	stop();
}

void
LoquendoRecogniserServer::stop()
{
	LOG4CXX_INFO(serverLogger, "stopping recognition");
	recognitionRunning = false;
	instance->stop();
//	audioSource->stop();

	for (vector<ClientPrx>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		Ice::Identity ident = (*it)->ice_getIdentity();
		LOG4CXX_DEBUG(serverLogger, "notifying listener " << ident.name << " on stopping"); 
		try {
			(*it)->onStop();
		}
		catch (...) {
			LOG4CXX_WARN(serverLogger, "failed to tell listener " << ident.name << " about the stop");
		}
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
LoquendoRecogniserServer::addClient(const Ice::Identity & ident, const Ice::Current & curr)
{
	LOG4CXX_INFO(serverLogger, "received addLoquendoClient(ident=(name=\"" << ident.name << "\", category=\""
			<< ident.category << "\"))");
	// TODO: check for errors
	ClientPrx listener = ClientPrx::uncheckedCast(curr.con->createProxy(ident))->ice_oneway();
	if (listener) {
		listeners.push_back(listener);
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
LoquendoRecogniserServer::shutdown(const Ice::Current & curr)
{
	LOG4CXX_INFO(serverLogger, "received shutdown()");
	shutdown();
}

void
LoquendoRecogniserServer::shutdown()
{
	stop();

	LOG4CXX_INFO(serverLogger, "notifying all listeners that the server is shutting down");
	for (vector<ClientPrx>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		Ice::Identity ident = (*it)->ice_getIdentity();
		LOG4CXX_DEBUG(serverLogger, "notifying listener " << ident.name); 
		try {
			(*it)->onUnregistrationFromServer("server shutdown");
		}
		catch (...) {
			LOG4CXX_WARN(serverLogger, "failed to tell listener " << ident.name << " to unregister");
		}
	}

	ic->shutdown();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

time::TimeValPtr
LoquendoRecogniserServer::getCurrentTime(const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received getCurrentTime()");
	return getCurrentTime();
}

time::TimeValPtr
LoquendoRecogniserServer::getCurrentTime()
{
	return getCurrentTimeOfDay();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

std::string
LoquendoRecogniserServer::getCWD(const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received getCWD()");
	char * cwd = get_current_dir_name();
	string s_cwd(cwd);
	free(cwd);
	return s_cwd;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

std::string
LoquendoRecogniserServer::getStatusString(const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received getStatusString()");

	stringstream ss(stringstream::in|stringstream::out);

	char * cwd = get_current_dir_name();
	string s_cwd(cwd);
	free(cwd);

	string s_status;
	switch (instance->getState()) {
		case LASRX_STATE_UNKNOWN:
			s_status = "UNKNOWN";
			break;
		case LASRX_STATE_IDLE:
			s_status = "IDLE";
			break;
		case LASRX_STATE_RECOGNIZE:
			s_status = "RECOGNIZE";
			break;
		case LASRX_STATE_RECORD:
			s_status = "RECORD";
			break;
		default:
			s_status = "other";
			break;
	}

	ss << "server_version: " << LOQICE_VERSION << endl;
	ss << "asr_version: " << session->getSystemVersionString() << endl;
	ss << "asr_info_string: " << session->getSystemInfo() << endl;
	ss << "working_directory: " << s_cwd << endl;
	ss << "session_file: " << sessionFile << endl;
	ss << "grammar_file: " << grammarFile << endl;
	ss << "status: " << s_status << endl;

	return ss.str();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

vector<Ice::Identity>
LoquendoRecogniserServer::getClients(const Ice::Current&)
{
	LOG4CXX_INFO(serverLogger, "received getClients()");

	vector<Ice::Identity> result;
	for (vector<ClientPrx>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		result.push_back((*it)->ice_getIdentity());
	}
	return result;
}
