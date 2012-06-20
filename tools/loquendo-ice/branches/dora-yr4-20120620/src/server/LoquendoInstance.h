#ifndef LOQUENDOINSTANCE_H__
#define LOQUENDOINSTANCE_H__  1

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

#include <LoqASR.h>
#include "InstanceException.h"

#include <utility>
#include <string>

typedef int LoquendoRejectionFlag;
typedef int LoquendoInstanceState;

class LoquendoInstance {
public:
	LoquendoInstance(lasrxHandleType handle);
	~LoquendoInstance();

	lasrxHandleType & getHandle() { return handle; }  // XXX: this should be hidden

	void stop();
	LoquendoInstanceState getState(); 

	std::string getId();
	unsigned int getExpectedSamplingFrequency();
	unsigned int getExpectedFrameRate();

	// Recognition results
	int getRRCPUTime();
	std::pair<int,int> getRRSpeechLimits();
	std::pair<std::string,std::string> getRRRONameAndRule();
	LoquendoRejectionFlag getRRRejectionAdvice();
	float getRRSignalToNoiseRatio();
	int getRRNumberOfHypothesis();

	std::pair<std::string,std::string> getRRHypothesisRONameAndRule(int idx);
	std::string getRRHypothesisString(int idx);
	float getRRHypothesisAcousticScore(int idx);
	float getRRHypothesisConfidence(int idx);
	int getRRHypothesisNumberOfWords(int idx);

	std::string getRRWordHypothesisString(int idx, int widx);
	std::string getRRWordHypothesisLanguage(int idx, int widx);
	float getRRWordHypothesisAcousticScore(int idx, int widx);
	float getRRWordHypothesisConfidence(int idx, int widx);
	std::pair<int,int> getRRWordHypothesisSegmentation(int idx, int widx);

	void clearROs();
	void addRO(const std::string & roName, const char * roRule);
	void setAudioDumpFileName(const std::string & filename);
	void setInstanceParam(const std::string & key, const std::string & value);
	void setCallbackGetEvent(lasrxCBGetEventType func);
	void setAudioGetEvent(lasrxCBAudioGetEventType func);

	void otfSetGrammarROGenerationDirectives(const std::string & grammarFile, const std::string & sourceName);
	std::string otfGenerateRO();
	void otfDeleteRO(const std::string & path);

	int recog();

	// has to be accessible by the recognition thread
	InstanceException newInstanceException(const std::string & function, int returnCode);

private:
	lasrxHandleType handle;
};

#endif
