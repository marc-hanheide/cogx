#ifndef MONITORCONSUMER_H__
#define MONITORCONSUMER_H__  1

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

#include "asr-loquendo.h"

namespace LoqASR = ::de::dfki::lt::tr::dialogue::asr::loquendo;

class MonitorListener : public LoqASR::Client {

public:
	MonitorListener(Ice::CommunicatorPtr ic);
	virtual ~MonitorListener();

	void onStart(const Ice::Current &);
	void onStop(const Ice::Current &);
	void onAudioSourceChange(const LoqASR::AudioSourcePtr & as, const Ice::Current &);
	void onEndOfStream(const Ice::Current &);
	void onGrammarChange(const std::string & filename, const Ice::Current &);
	void onRecognitionResult(const LoqASR::result::RecognitionResultPtr & rr, const Ice::Current &);
	void onUnregistrationFromServer(const std::string & reason, const Ice::Current &);

private:
	Ice::CommunicatorPtr ic;

	void receiveNoRecognitionResult(const LoqASR::result::NoRecognitionResultPtr & nrr);
	void receiveNBestList(const LoqASR::result::NBestListPtr & rr);
	void receiveWordLattice(const LoqASR::result::WordLatticePtr & wl);
};


#endif
