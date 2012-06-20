#ifndef LOQUENDORECOGNISERSERVER_H__
#define LOQUENDORECOGNISERSERVER_H__  1

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

#include <Ice/Ice.h>
#include <pthread.h>

#include <LoqASR.h>
#include "asr-loquendo.h"
#include "AudioSource.h"

#include "LoquendoSession.h"
#include "LoquendoInstance.h"
#include "RecognitionThread.h"

#include <string>
#include <vector>

namespace LoqASR = ::de::dfki::lt::tr::dialogue::asr::loquendo;

class LoquendoRecogniserServer : public LoqASR::Server {

public:
//	LoquendoRecogniser();
	LoquendoRecogniserServer(const Ice::CommunicatorPtr & ic_, const std::string & sessionFile_, const std::string & grammarFile_, bool doAudiodumps_, const std::string & audiodumpPrefix_, const std::string & audiodumpSuffix_, bool exportWordLattices);
	virtual ~LoquendoRecogniserServer();

	virtual void setAudioSource(const LoqASR::AudioSourcePtr & as, const Ice::Current&);
	virtual void setGrammarFile(const std::string & filename, const Ice::Current&);

	virtual void start(const Ice::Current&);
	virtual void start();
	virtual void stop(const Ice::Current&);
	virtual void stop();

	void shutdown(const Ice::Current&);
	void shutdown();

	virtual void addClient(const Ice::Identity & ident, const Ice::Current&);

	bool doingAudiodumps() { return doAudiodumps; }
	const std::string & getAudiodumpPrefix() { return audiodumpPrefix; }
	const std::string & getAudiodumpSuffix() { return audiodumpSuffix; }
	bool isRecognitionRunning() { return recognitionRunning; }

	LoqASR::time::TimeValPtr getCurrentTime(const Ice::Current&);
	std::string getCWD(const Ice::Current&);
	std::string getStatusString(const Ice::Current&);
	std::vector<Ice::Identity> getClients(const Ice::Current&);

private:

	friend class RecognitionThread;

	LoqASR::LoquendoException newLoquendoException(const char * functionName, int returnCode);
	void loadPCMCaptureAudioSource();
	void loadRAWFileAudioSource(const std::string & path);
	void setAudioSource(const LoqASR::AudioSourcePtr & as);
	int buildRO(const std::string & grammarFile, const std::string & sourceName);
	int recog(const std::string & roName, const char * roRule);
	LoqASR::result::NBestListPtr getNBestList(int maxHypos);
	LoqASR::result::WordLatticePtr getWordLattice();
	void setupLatticeExport();

	LoqASR::time::TimeValPtr getCurrentTime();

	void throwInstanceException(const InstanceException & e);

	LoquendoSession * session;
	LoquendoInstance * instance;

	AudioSource * audioSource;

	std::string sessionFile;
	std::string grammarFile;

	Ice::CommunicatorPtr ic;

	std::vector<LoqASR::ClientPrx> listeners;

	bool doAudiodumps;
	std::string audiodumpPrefix;
	std::string audiodumpSuffix;

	bool exportWordLattices;
	std::string latticeFilename;

	bool recognitionRunning;
	RecognitionThread * recognitionThread;
	pthread_mutex_t recogMutex;
};

#endif
