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

#include <unistd.h>
#include <signal.h>

#include "LoqASR.h"
#include "asr-loquendo.h"

#include "RecognitionThread.h"
#include "LoquendoRecogniserServer.h"
#include "InstanceException.h"

#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace log4cxx;
using namespace LoqASR;
using namespace LoqASR::result;

RecognitionThread::RecognitionThread(LoggerPtr logger_, const LoquendoRecogniserServer & server_)
: server((LoquendoRecogniserServer *) &server_), logger(logger_)
{ 
	server = (LoquendoRecogniserServer *) &server_;
	LOG4CXX_INFO(logger, "spawning the recognition thread");
	pthread_create(&id, NULL, entryPoint, this);
}

RecognitionThread::~RecognitionThread()
{
	LOG4CXX_INFO(logger, "killing the recognition thread");
	pthread_kill(id, 9);
}

void *
RecognitionThread::entryPoint(void * pthis)
{
	return (void *) ((RecognitionThread *) pthis)->run();
}

int
RecognitionThread::run()
{
	static int index = 0;
	LOG4CXX_INFO(logger, "about to start recognition");

	pthread_mutex_lock(&server->recogMutex);

	while (server->recognitionRunning) {

		if (server->doAudiodumps) {
			// generate the audio dump file name
			stringstream fn(stringstream::in|stringstream::out);
			fn << server->getAudiodumpPrefix() << "-" << getpid() << "-" << setfill('0') << setw(4) << index << server->getAudiodumpSuffix();
			index++;

			LOG4CXX_INFO(logger, "will write audiodump in `" << fn.str() << "'");
			server->instance->setAudioDumpFileName(fn.str());
		}

		LOG4CXX_DEBUG(logger, "recognition start");
		int rc = server->instance->recog();
		LOG4CXX_DEBUG(logger, "recognition ended (return code = " << rc << ")");

		RecognitionResultPtr rr = NULL;
		bool isEOS = false;
		bool unregisterAll = false;
		string unregisterReason;

		switch (rc) {
			case LASRX_RETCODE_OK:
				// got results -> send them over
				if (server->exportWordLattices) {
					LOG4CXX_DEBUG(logger, "recognition result = word lattice");
					rr = server->getWordLattice();
				}
				else {
					LOG4CXX_DEBUG(logger, "recognition result = n-best list");
					rr = server->getNBestList(10);
				}
				break;

			case LASRX_RETCODE_NO_RESULTS:
				// no result, but notify
				LOG4CXX_DEBUG(logger, "recognition result = no-recognition");
				rr = new NoRecognitionResult();
				break;

			case LASRX_RETCODE_STOPPED:
				// no result, no notification
				LOG4CXX_DEBUG(logger, "reason for recognition end: stopped");
				break;

			case LASRX_RETCODE_AUDIO:
				LOG4CXX_DEBUG(logger, "reason for recognition end: audio event (end of stream)");
				isEOS = true;
				server->recognitionRunning = false;
				break;

			case LASRX_RETCODE_OVERFLOW:
				LOG4CXX_WARN(logger, "LASRX_RETCODE_OVERFLOW received, discarding this run");
				break;

			case LASRX_RETCODE_EPD_MAX_LENGTH:
				LOG4CXX_WARN(logger, "LASRX_RETCODE_EPD_MAX_LENGTH received, discarding this run");
				break;

			default:
				InstanceException e = server->instance->newInstanceException("lasrxRecog", rc);
				server->stop();
				// FIXME: clean up!
				LOG4CXX_ERROR(logger, "error in recognition: " << e.function << "(): " << e.message);
				unregisterAll = true;
				unregisterReason = e.message;
				break;
		}

		if (isEOS) {
			for (vector<ClientPrx>::iterator it = server->listeners.begin(); it != server->listeners.end(); it++) {
				try {
					(*it)->onEndOfStream();
				}
				catch (...) {
					LOG4CXX_WARN(logger, "failed to tell the listener about the EOS");
				}
			}
		}
		else {
			// not an end-of-stream

			if (rr != NULL) {

				vector<ClientPrx> newListeners;

				if (unregisterAll) {
					LOG4CXX_INFO(logger, "unregistering all listeners");
					for (vector<ClientPrx>::iterator it = server->listeners.begin(); it != server->listeners.end(); it++) {
						try {
							(*it)->onUnregistrationFromServer(unregisterReason);
						}
						catch (...) {
							LOG4CXX_WARN(logger, "failed to tell the listener to unregister");
						}
					}
					server->listeners = newListeners;
				}
				else if (rr) {
					for (vector<ClientPrx>::iterator it = server->listeners.begin(); it != server->listeners.end(); it++) {
						bool ok = true;
						try {
							(*it)->onRecognitionResult(rr);
						}
						catch (Ice::Exception ex) {
							Ice::Identity ident = (*it)->ice_getIdentity();
							LOG4CXX_WARN(logger, "unregistering the listener " << ident.name << ": " << ex);
							ok = false;
						}
						if (ok) {
							newListeners.push_back(*it);
						}
					}
				}
				server->listeners = newListeners;

				if (server->listeners.empty()) {
					LOG4CXX_INFO(logger, "no listener registered, stopping");
					server->stop();
				}
			}
			else {
				LOG4CXX_DEBUG(logger, "got a null result, ignoring");
			}
		}
	}
	LOG4CXX_INFO(logger, "recognition over, freeing resources");
	pthread_mutex_unlock(&server->recogMutex);
	server->stop();

	pthread_exit(NULL);
}

bool
RecognitionThread::isRunning()
{
	return pthread_kill(id, 0) != ESRCH;
}
