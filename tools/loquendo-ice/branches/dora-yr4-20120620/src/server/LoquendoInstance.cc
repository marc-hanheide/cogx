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

#include <stdlib.h>
#include "LoquendoInstance.h"

#include <iostream>
#include <utility>
#include <string>

#include <log4cxx/logger.h>

using namespace std;
using namespace log4cxx;

LoggerPtr instLogger(Logger::getLogger("loquendo-asr.instance"));

InstanceException
LoquendoInstance::newInstanceException(const std::string & function, int returnCode)
{
	char * str;
	lasrxGetErrorMessage(handle, &str);
	string message(str);
	lasrxFree(str);
	return InstanceException(function, returnCode, message);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

LoquendoInstance::LoquendoInstance(lasrxHandleType handle_)
: handle(handle_)
{
	LOG4CXX_DEBUG(instLogger, "created instance "
			<< "0x" << hex << handle << dec
			<< " (" << getId() << ")"
			);
}

LoquendoInstance::~LoquendoInstance()
{
	if (handle) {
		LOG4CXX_TRACE(instLogger, "deleting the instance");
        lasrxDeleteInstance(handle);
		handle = NULL;
	}
	LOG4CXX_TRACE(instLogger, "deleted");
}

void
LoquendoInstance::stop()
{
	LOG4CXX_INFO(instLogger, "stopping");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxStop(handle)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxStop", rc);
	}
}

LoquendoInstanceState
LoquendoInstance::getState()
{
	int state;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxGetState(handle, &state)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxGetState", rc);
	}
	return state;
}

string
LoquendoInstance::getId()
{
	lasrxResultType rc = LASRX_RETCODE_OK;
	lasrxStringType id;

	if ((rc = lasrxGetInstanceId(handle, &id)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxGetInstanceId", rc);
	}

	string out(id);
    lasrxFree(id);

	return out;
}

unsigned int
LoquendoInstance::getExpectedSamplingFrequency()
{
	unsigned int expSampleFreq;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxGetSamplingFrequency(handle, &expSampleFreq)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxGetSamplingFrequency", rc);
	}
	return expSampleFreq;
}

unsigned int
LoquendoInstance::getExpectedFrameRate()
{
	unsigned int expFrameRate;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxGetFrameRate(handle, &expFrameRate)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxGetFrameRate", rc);
	}
	return expFrameRate;
}

int
LoquendoInstance::getRRCPUTime()
{
	int cpu;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetCPUTime(handle, &cpu)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetCPUTime", rc);
	}
	return cpu;
}

pair<int,int>
LoquendoInstance::getRRSpeechLimits()
{
	int start, end;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetSpeechLimits(handle, LASRX_UNITS_FRAMES, &start, &end)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetSpeechLimits", rc);
	}
	return make_pair(start, end);
}

pair<string,string>
LoquendoInstance::getRRRONameAndRule()
{
	lasrxStringType roName;
	lasrxEncodedStringType roRule;

	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetRONameAndRule(handle, &roName, &roRule)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetRONameAndRule", rc);
	}

	string name((const char *) roName);
	string rule((const char *) roRule);
	lasrxFree(roName);
	lasrxFree(roRule);

	return make_pair(name, rule);
}

LoquendoRejectionFlag
LoquendoInstance::getRRRejectionAdvice()
{
	int rejection;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetRejectionAdvice(handle, &rejection)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetRejectionAdvice", rc);
	}
	return rejection;
}

float
LoquendoInstance::getRRSignalToNoiseRatio()
{
	float snr;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetSignalToNoiseRatio(handle, &snr)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetSignalToNoiseRatio", rc);
	}
	return snr;
}

int
LoquendoInstance::getRRNumberOfHypothesis()
{
	int numHypos = 0;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetNumberOfHypothesis(handle, &numHypos)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetNumberOfHypothesis", rc);
	}
	return numHypos;
}

pair<string,string>
LoquendoInstance::getRRHypothesisRONameAndRule(int idx)
{
	lasrxStringType roName;
	lasrxEncodedStringType roRule;

	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetHypothesisRONameAndRule(handle, idx, &roName, &roRule)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetHypothesisRONameAndRule", rc);
	}

	string name((const char *) roName);
	string rule((const char *) roRule);
	lasrxFree(roName);
	lasrxFree(roRule);

	return make_pair(name, rule);
}

string
LoquendoInstance::getRRHypothesisString(int idx)
{
	lasrxEncodedStringType encStr;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetHypothesisString(handle, idx, &encStr)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetHypothesisString", rc);
	}
	string out((const char *) encStr);
	lasrxFree(encStr);
	return out;
}

float
LoquendoInstance::getRRHypothesisAcousticScore(int idx)
{
	float acousticScore;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetHypothesisAcousticScore(handle, idx, &acousticScore)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetHypothesisAcousticScore", rc);
	}
	return acousticScore;
}

float
LoquendoInstance::getRRHypothesisConfidence(int idx)
{
	float confidence;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetHypothesisConfidence(handle, idx, &confidence)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetHypothesisConfidence", rc);
	}
	return confidence;
}

int
LoquendoInstance::getRRHypothesisNumberOfWords(int idx)
{
	int numWords;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetHypothesisNumberOfWords(handle, idx, &numWords)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetHypothesisNumberOfWords", rc);
	}
	return numWords;
}

string
LoquendoInstance::getRRWordHypothesisString(int idx, int widx)
{
	lasrxEncodedStringType encStr;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetWordHypothesisString(handle, idx, widx, &encStr)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetWordHypothesisString", rc);
	}
	string out((const char *) encStr);
	lasrxFree(encStr);
	return out;
}

string
LoquendoInstance::getRRWordHypothesisLanguage(int idx, int widx)
{
	lasrxStringType str;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetWordHypothesisLanguage(handle, idx, widx, &str)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetWordHypothesisLanguage", rc);
	}
	string out((const char *) str);
	lasrxFree(str);
	return out;
}

float
LoquendoInstance::getRRWordHypothesisAcousticScore(int idx, int widx)
{
	float acousticScore;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetWordHypothesisAcousticScore(handle, idx, widx, &acousticScore)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetWordHypothesisAcousticScore", rc);
	}
	return acousticScore;
}

float
LoquendoInstance::getRRWordHypothesisConfidence(int idx, int widx)
{
	float confidence;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetWordHypothesisConfidence(handle, idx, widx, &confidence)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetWordHypothesisConfidence", rc);
	}
	return confidence;
}

pair<int,int>
LoquendoInstance::getRRWordHypothesisSegmentation(int idx, int widx)
{
	int start, end;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxRRGetWordHypothesisSegmentation(handle, idx, widx, LASRX_UNITS_FRAMES, &start, &end)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxRRGetWordHypothesisSegmentation", rc);
	}
	return make_pair(start, end);
}

void
LoquendoInstance::clearROs()
{
	LOG4CXX_TRACE(instLogger, "clearROs()");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxClearROs(handle)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxClearROs", rc);
	}
}

void
LoquendoInstance::addRO(const std::string & roName, const char * roRule)
{
	LOG4CXX_TRACE(instLogger, "addRO(\"" << roName << "\", \"" << roRule << "\")");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxAddRO(handle, (char *) roName.c_str(), (char *) roRule)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxAddRO", rc);
	}
}

void
LoquendoInstance::setAudioDumpFileName(const std::string & filename)
{
	LOG4CXX_TRACE(instLogger, "setAudioDumpFileName(\"" << filename << "\")");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetAudioDumpFileName(handle, (char *) filename.c_str())) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxSetAudioDumpFileName", rc);
	}
}

void
LoquendoInstance::setInstanceParam(const std::string & key, const std::string & value)
{
	LOG4CXX_TRACE(instLogger, "setInstanceParam(\"" << key << "\", \"" << value << "\")");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetInstanceParam(handle, (char *) key.c_str(), (char *) value.c_str())) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxSetInstanceParam", rc);
	}
}

void
LoquendoInstance::setCallbackGetEvent(lasrxCBGetEventType func)
{
	LOG4CXX_TRACE(instLogger, "setCallbackGetEvent(...)");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetCallbackGetEvent(handle, func)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxSetCallbackGetEvent", rc);
	}
}

void
LoquendoInstance::setAudioGetEvent(lasrxCBAudioGetEventType func)
{
	LOG4CXX_TRACE(instLogger, "setAudioGetEvent(...)");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetAudioGetEvent(handle, func)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxSetAudioGetEvent", rc);
	}
}

void
LoquendoInstance::otfSetGrammarROGenerationDirectives(const std::string & grammarFile, const std::string & sourceName)
{
	LOG4CXX_TRACE(instLogger, "otfSetGrammarROGenerationDirectives(\"" << grammarFile << "\", \"" << sourceName << "\")");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxOTFSetGrammarROGenerationDirectives(handle, (char *) grammarFile.c_str(), NULL, (char *) LASRX_RPNAME_DEFAULT, NULL, 0, NULL, LASRX_FALSE, (char *) sourceName.c_str(), NULL)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxOTFSetGrammarROGenerationDirectives", rc);
	}
}

string
LoquendoInstance::otfGenerateRO()
{
	LOG4CXX_TRACE(instLogger, "otfGenerateRO()");
	lasrxStringType szOpId;
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxOTFGenerateRO(handle, LASRX_RUNMODE_BLOCKING, &szOpId)) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxOTFGenerateRO", rc);
	}
	string opId(szOpId);
	lasrxFree(szOpId);

	return opId;
}

void
LoquendoInstance::otfDeleteRO(const std::string & path)
{
	LOG4CXX_TRACE(instLogger, "otfDeleteRO()");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxOTFDeleteRO(handle, (char *) path.c_str())) != LASRX_RETCODE_OK) {
		throw newInstanceException("lasrxOTFDeleteRO", rc);
	}
}

int
LoquendoInstance::recog()
{
	LOG4CXX_TRACE(instLogger, "recog()");
	lasrxStringType szOpId;
	lasrxResultType rc = lasrxRecog(handle, LASRX_RUNMODE_BLOCKING, &szOpId);
	lasrxFree(szOpId);

	return rc;
}
