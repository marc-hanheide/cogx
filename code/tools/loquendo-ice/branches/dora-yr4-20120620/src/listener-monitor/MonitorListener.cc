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

#include "MonitorListener.h"

#include <Ice/Ice.h>

#include "asr-loquendo.h"

#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <time.h>

#include "TtyUtils.h"

using namespace std;
using namespace LoqASR;
using namespace LoqASR::result;

string
rejectionAdviceToString(RejectionFlag f)
{
	switch (f) {
	case RecognitionFalse:
		return "FALSE";
	case RecognitionNomatch:
		return "NOMATCH";
	default:
		return "[unknown]";
	}
}

string
timeValToString(time::TimeValPtr tv)
{
	time_t ts = tv->sec;
	struct tm * tm = localtime(&ts);

	stringstream nss(stringstream::in|stringstream::out);

	nss << tm->tm_year + 1900;
	nss << "-" << setw(2) << setfill('0') << tm->tm_mon + 1 << resetiosflags(ios_base::showbase);
	nss << "-" << setw(2) << setfill('0') << tm->tm_mday << resetiosflags(ios_base::showbase);
	nss << "  ";
	nss << setw(2) << setfill('0') << tm->tm_hour << resetiosflags(ios_base::showbase);
	nss << ":" << setw(2) << setfill('0') << tm->tm_min << resetiosflags(ios_base::showbase);
	nss << ":" << setw(2) << setfill('0') << tm->tm_sec << resetiosflags(ios_base::showbase);
	nss << "." << setw(3) << setfill('0') << (int)(tv->usec / 1000) << resetiosflags(ios_base::showbase);

	return nss.str();
}

MonitorListener::MonitorListener(Ice::CommunicatorPtr ic_)
: ic(ic_)
{
	cerr << tty::green << "* monitor initialised" << tty::dcol << endl;
}

MonitorListener::~MonitorListener()
{
	cerr << tty::green << "* monitor destroyed" << tty::dcol << endl;
}

void
MonitorListener::onRecognitionResult(const RecognitionResultPtr & rr, const Ice::Current &)
{
	cerr << tty::green << "* received a recognition result" << tty::dcol << endl;

	if (NoRecognitionResultPtr noRR = NoRecognitionResultPtr::dynamicCast(rr)) {
		receiveNoRecognitionResult(noRR);
	}
	else if (NBestListPtr nBestListRR = NBestListPtr::dynamicCast(rr)) {
		receiveNBestList(nBestListRR);
	}
	else if (WordLatticePtr wordLatticeRR = WordLatticePtr::dynamicCast(rr)) {
		receiveWordLattice(wordLatticeRR);
	}
	else {
		cerr << tty::red << "  unknown type!" << tty::dcol << endl;
	}
}

void
MonitorListener::onUnregistrationFromServer(const string & reason, const Ice::Current &)
{
	cerr << tty::red << "* unregistered: \"" << reason << "\"" << tty::dcol << endl;
	ic->shutdown();
}

void
MonitorListener::onStart(const Ice::Current &)
{
	cerr << tty::yellow << "* [start]" << tty::dcol << endl;
}

void
MonitorListener::onStop(const Ice::Current &)
{
	cerr << tty::yellow << "* [stop]" << tty::dcol << endl;
}

void
MonitorListener::onAudioSourceChange(const AudioSourcePtr & as, const Ice::Current &)
{
	string s_as = "UNKNOWN";

	if (PulseAudioPCMCapturePtr as_pcm = PulseAudioPCMCapturePtr::dynamicCast(as)) {
		s_as = "PCM capture";
	}
	else if (RAWFilePtr as_raw = RAWFilePtr::dynamicCast(as)) {
		s_as = "\"" + as_raw->path + "\" (RAW file)";
	}

	cerr << tty::yellow << "* [audiosource: " << s_as << "]" << tty::dcol << endl;
}

void
MonitorListener::onEndOfStream(const Ice::Current &)
{
	cerr << tty::yellow << "* [eos]" << tty::dcol << endl;
}

void
MonitorListener::onGrammarChange(const string & filename, const Ice::Current &)
{
	cerr << tty::yellow << "* [grammar: \"" << filename << "\"]" << tty::dcol << endl;
}

void
MonitorListener::receiveNBestList(const NBestListPtr & rr)
{
	cerr << "  type = NBestList" << endl;
	cerr << "  time anchor = " << timeValToString(rr->timeAnchor) << endl;
	cerr << "  signal-to-noise ratio = " << rr->snr << endl;
	cerr << "  rejection advice = " << rejectionAdviceToString(rr->rejectionAdvice) << endl;

	int i = 1;
	for (vector<HypothesisPtr>::const_iterator it = rr->hypos.begin(); it != rr->hypos.end(); it++) {
		HypothesisPtr hypo = *it;

		cerr << "    " << setw(2) << i << " ... ";
//		cerr << "  \"" << hypo->str << "\"" << endl;

		cerr << tty::white;
		for (vector<WordHypothesisPtr>::const_iterator wit = hypo->words.begin(); wit != hypo->words.end(); wit++) {
			WordHypothesisPtr whypo = *wit;
//			cerr << "\"" << whypo->word << "\" ";
			cerr << whypo->word << " ";
		}
		cerr << tty::dcol;
		cerr << " (score = " << hypo->acousticScore << ", cfd = " << hypo->confidence << ")";
		cerr << endl;
		i++;
	}
}

void
MonitorListener::receiveWordLattice(const WordLatticePtr & wl)
{
	cerr << "  type = WordLattice" << endl;
	cerr << "  time anchor = " << timeValToString(wl->timeAnchor) << endl;

	if (wl) {
		cerr << "  nodes = {\n";
		vector<NodePtr>::iterator nit;
		for (nit = wl->nodes.begin(); nit != wl->nodes.end(); nit++) {
			cerr << "    [" << (*nit)->id << "]: t=" << (*nit)->time << ", f=" << (*nit)->frame << endl;
		}
		cerr << "  }\n";

		cerr << "  links = {\n";
		vector<LinkPtr>::iterator lit;
		for (lit = wl->links.begin(); lit != wl->links.end(); lit++) {
			cerr << "    (" << (*lit)->id << "): [" << (*lit)->start << "] -> [" << (*lit)->end << "]: \""
					<< tty::white << (*lit)->word << tty::dcol
					<< "\" (c=" << (*lit)->confidence << ", w="
					<< (*lit)->searchScore << ", p=" << (*lit)->sequenceScore << ", a="
					<< (*lit)->acoustic << ")" << endl;
		}
		cerr << "  }\n";
	}
	else {
		cerr << tty::red << "  NULL" << tty::dcol;
	}
}

void
MonitorListener::receiveNoRecognitionResult(const NoRecognitionResultPtr & rr)
{
	cerr << "  type = NoRecognitionResult" << endl;
}
