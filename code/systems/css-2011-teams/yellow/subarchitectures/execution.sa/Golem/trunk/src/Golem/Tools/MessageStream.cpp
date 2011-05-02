/** @file MessageStream.cpp
 * 
 * Message stream.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/MessageStream.h>
#include <Golem/Tools/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

MessageStream::MessageStream() {
}

bool MessageStream::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgMessageStreamInvalidDesc(Message::LEVEL_CRIT, "MessageStream::create(): Invalid description");

	filter = desc.filter;
	return true;
}

void MessageStream::write(SecTmReal time, U32 thread, Message::Level level, Message::Code code, const char* format, va_list argptr) {
	if (filter == NULL || (*filter)(time, thread, level, code)) {
		Message::Ptr pMessage(new Message);
		pMessage->create(time, thread, level, code, format, argptr);
		write(pMessage);
	}
}

void MessageStream::write(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(Message::TIME_UNDEF, Message::THREAD_UNDEF, Message::LEVEL_UNDEF, Message::CODE_UNDEF, format, argptr);		
	va_end(argptr);
}

void MessageStream::write(Message::Level level, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(Message::getCurrentTime(), Message::getCurrentThread(), level, Message::CODE_UNDEF, format, argptr);		
	va_end(argptr);
}

void MessageStream::write(Message::Level level, Message::Code code, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	write(Message::getCurrentTime(), Message::getCurrentThread(), level, code, format, argptr);
	va_end(argptr);
}

//------------------------------------------------------------------------------

BufferedStream::~BufferedStream() {
	terminate = true;
	evQueue.set(true);
	if (!thread.join(threadTimeOut)) {
		// ignore
	}
}

bool BufferedStream::create(const Desc &desc) {
	MessageStream::create(desc); // throws

	cstr = desc.cstr;
	cppstr = desc.cppstr;
	messages.reserve(desc.queueSize);
	threadTimeOut = desc.threadTimeOut;
	terminate = false;

	std::ios::sync_with_stdio(false);

	if (!thread.start(this))
		throw MsgMessageStreamThreadLaunch(Message::LEVEL_CRIT, "BufferedStream::create(): Unable to launch thread");

	if (!thread.setPriority(desc.threadPriority)) {
		// ignore
	}

	return true;
}

void BufferedStream::run() {
	for (;;) {
		(void)evQueue.wait();

		Message::Ptr message;
		{
			CriticalSectionWrapper csw(csQueue);
			if (messages.empty()) {
				if (terminate)
					break;
				evQueue.set(false);
				continue;
			}
			message = messages.front();
			messages.pop_front();
		}
		{
			CriticalSectionWrapper csw(csStream);
			if (cstr)
				fprintf(cstr, "%s\n", message->str().c_str());
			if (cppstr)
				*cppstr << message->str() << std::endl;
		}
	}
}

void BufferedStream::write(Message::Ptr message) {
	CriticalSectionWrapper csw(csQueue);
	messages.push_back(message);
	evQueue.set(true);
}

void BufferedStream::setCStream(FILE* cstr) {
	CriticalSectionWrapper csw(csStream);
	this->cstr = cstr;
}

void BufferedStream::setCppStream(std::ostream* cppstr) {
	CriticalSectionWrapper csw(csStream);
	this->cppstr = cppstr;
}

//------------------------------------------------------------------------------
