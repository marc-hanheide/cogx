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
#include "LoquendoSession.h"

#include <iostream>
#include <string>

#include <log4cxx/logger.h>

using namespace std;
using namespace log4cxx;

LoggerPtr sessionLogger(Logger::getLogger("loquendo-asr.session"));

SessionException
LoquendoSession::newSessionException(const std::string & function, int returnCode)
{
	char * str;
	lasrxGetErrorMessage(handle, &str);
	string message(str);
	lasrxFree(str);
	return SessionException(function, returnCode, message);
}

LoquendoSession::LoquendoSession(const char * sessionFile)
: handle(NULL)
{
	LOG4CXX_TRACE(sessionLogger, "creating a new session using '" << sessionFile << "' as the session file");
    lasrxResultType rc = LASRX_RETCODE_OK;
    if ((rc = lasrxNewSession((char *) sessionFile, NULL, &handle)) != LASRX_RETCODE_OK) {
		throw newSessionException("lasrxNewSession", rc);
    }
}

LoquendoSession::~LoquendoSession()
{
	LOG4CXX_TRACE(sessionLogger, "deallocating the session");
	if (handle) {
        lasrxDeleteSession(handle);
		handle = NULL;
	}
}

LoquendoInstance *
LoquendoSession::newInstance()
{
	LOG4CXX_TRACE(sessionLogger, "creating a new instance");
	lasrxResultType rc = LASRX_RETCODE_OK;
	lasrxHandleType instHandle = NULL;
    if ((rc = lasrxNewInstance(handle, NULL, LASRX_FALSE, &instHandle)) != LASRX_RETCODE_OK) {
		throw newSessionException("lasrxNewSession", rc);
    }
	return new LoquendoInstance(instHandle);
}

void
LoquendoSession::setAudioMode(unsigned int sampleFormat)
{
	string mstr;
	switch (sampleFormat) {
		case LASRX_SAMPLES_UL:
			mstr = "u-law";
			break;
		case LASRX_SAMPLES_AL:
			mstr = "A-law";
			break;
		case LASRX_SAMPLES_LIN16:
			mstr = "16-bit linear PCM";
			break;
		default:
			mstr = "UNKNOWN";
			break;
	}
	LOG4CXX_TRACE(sessionLogger, "setting audio mode to " << mstr);
	lasrxResultType rc = LASRX_RETCODE_OK;
    if ((rc = lasrxSetAudioMode(handle, sampleFormat)) != LASRX_RETCODE_OK) {
		throw newSessionException("lasrxSetAudioMode", rc);
    }
}

unsigned int
LoquendoSession::getAudioMode()
{
	lasrxResultType rc = LASRX_RETCODE_OK;
	int sampleFormat;
    if ((rc = lasrxGetAudioMode(handle, &sampleFormat)) != LASRX_RETCODE_OK) {
		throw newSessionException("lasrxGetAudioMode", rc);
    }
	return sampleFormat;
}

std::string
LoquendoSession::getSystemVersionString()
{
	lasrxResultType rc = LASRX_RETCODE_OK;
	lasrxStringType versionString;
	int versionInt;
    if ((rc = lasrxGetVersion(&versionString, &versionInt)) != LASRX_RETCODE_OK) {
		throw newSessionException("lasrxGetVersion", rc);
    }
	std::string out(versionString);
	lasrxFree(versionString);
	return out;
}

std::string
LoquendoSession::getSystemInfo()
{
	lasrxResultType rc = LASRX_RETCODE_OK;
	lasrxStringType infoString;
    if ((rc = lasrxQuery(handle, LASRX_FORMAT_TEXT, &infoString)) != LASRX_RETCODE_OK) {
		throw newSessionException("lasrxQuery", rc);
    }
	std::string out(infoString);
	lasrxFree(infoString);
	return out;
}
