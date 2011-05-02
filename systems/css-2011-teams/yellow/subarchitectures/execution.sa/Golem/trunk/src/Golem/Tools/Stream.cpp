/** @file Stream.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Defs/Assert.h>
#include <Golem/Defs/Types.h>
#include <Golem/Tools/Stream.h>
#include <Golem/Tools/Msg.h>
#include <memory.h>

#ifdef WIN32
#elif LINUX
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Stream::Stream() {
}

Stream::~Stream() {
}

void Stream::readBytes(void *dst, size_t size) const {
	throw MsgStream(Message::LEVEL_CRIT, "Stream::readBytes(): not implemented");
}

void Stream::writeBytes(const void *src, size_t size) {
	throw MsgStream(Message::LEVEL_CRIT, "Stream::writeBytes(): not implemented");
}

void Stream::resetRead() const {
	throw MsgStream(Message::LEVEL_CRIT, "Stream::resetRead(): not implemented");
}

void Stream::resetWrite() {
	throw MsgStream(Message::LEVEL_CRIT, "Stream::resetWrite(): not implemented");
}

size_t Stream::lastBytes() const {
	throw MsgStream(Message::LEVEL_CRIT, "Stream::lastBytes(): not implemented");
	return 0;
}

//------------------------------------------------------------------------------

template <> void Stream::read(std::string* str, size_t length) const {
	ASSERT(str)
	for (size_t i = 0; i < length; i++) {
		char c;
		readBytes(&c, sizeof(char));
		if (c == '\0')
			return;
		str->push_back(c);
	}
}

template <> void Stream::read(std::string& str) const {
	read(&str, -1);
}

template <> void Stream::write(const std::string* str, size_t length) {
	ASSERT(str)
	const size_t size = std::min(str->size(), length);
	writeBytes(str->data(), sizeof(char)*size);
	const char c = '\0';
	writeBytes(&c, sizeof(char));
}

template <> void Stream::write(const std::string& str) {
	write(&str, -1);
}

//------------------------------------------------------------------------------

MemoryWriteStream::~MemoryWriteStream() {
	resetWrite();
}

void MemoryWriteStream::writeBytes(const void *src, size_t size) {
	buffer.push_back(NULL);
	buffer.back() = new U8 [size];
	memcpy(buffer.back(), src, size);
}

void MemoryWriteStream::resetWrite() {
	for (Buffer::const_iterator i = buffer.begin(); i != buffer.end(); i++)
		delete [] (U8*)*i;
	buffer.clear();
}

//------------------------------------------------------------------------------

MemoryReadStream::MemoryReadStream(const Buffer &buffer) : buffer(buffer) {
	resetRead();
}

void MemoryReadStream::readBytes(void *dst, size_t size) const {
	if (ptr == buffer.end())
		throw MsgStreamRead(Message::LEVEL_CRIT, "MemoryReadStream::readBytes(): Unable to read from the stream");
	memcpy(dst, *ptr++, size);
}

void MemoryReadStream::resetRead() const {
	ptr = buffer.begin();
}

//------------------------------------------------------------------------------

void golem::mkdir(const char *_dir) {
	std::string dir(_dir);
	bool bName = false;
	for (U32 i = 0; i < dir.length(); i++) {
#ifdef WIN32
		bool bDir = dir.at(i) == '/' || dir.at(i) == '\\';
#else
		bool bDir = dir.at(i) == '/';
#endif
		if (!bDir)
			bName = true;
#ifdef WIN32
		if (dir.at(i) == ':')
			bName = false;
#else
		if (dir.at(i) == '.' || dir.at(i) == '~')
			bName = false;
#endif
		if (bName && bDir) {
			dir.at(i) = '\0';
#ifdef WIN32
			if (CreateDirectory(dir.c_str(), NULL) == FALSE)
				if (GetLastError() != ERROR_ALREADY_EXISTS)
					throw MsgStreamDirCreateFail(Message::LEVEL_CRIT, "FileStream::mkdir(): Unable to create directory \"%s\"", dir.c_str());
#else
			if (::mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1)
				if (errno != EEXIST) // directory exists
					throw MsgStreamDirCreateFail(Message::LEVEL_CRIT, "FileStream::mkdir(): Unable to create directory \"%s\": %s", dir.c_str(), strerror(errno));
#endif
			dir.at(i) = '/';
		}
	}
}

//------------------------------------------------------------------------------

FileStream::FileStream(const char *filePath, std::ios::openmode mode) : filePath(filePath) {
//	if (mode & std::ios::out)
//		mkdir(filePath);

	filestream.open(filePath, mode);
	if (filestream.fail())
		throw MsgStreamFileOpenFail(Message::LEVEL_CRIT, "FileStream::FileStream(): Unable to open \"%s\"", filePath);
}

FileStream::~FileStream() {
	filestream.close();
}

void FileStream::readBytes(void *dst, size_t size) const {
	if (size <= 0)
		return;
	filestream.read((char*)dst, (std::streamsize)size*sizeof(char));
	if (filestream.fail())
		throw MsgStreamRead(Message::LEVEL_CRIT, "FileStream::readBytes(): Failed to read from \"%s\"", filePath);
}

void FileStream::writeBytes(const void *src, size_t size) {
	if (size <= 0)
		return;
	filestream.write((const char*)src, (std::streamsize)size*sizeof(char));
	if (filestream.fail())
		throw MsgStreamWrite(Message::LEVEL_CRIT, "FileStream::writeBytes(): Failed to write to \"%s\"", filePath);
}

void FileStream::resetRead() const {
	filestream.seekg(0, std::ios::beg);
}

void FileStream::resetWrite() {
	filestream.seekp(0, std::ios::beg);
}

size_t FileStream::lastBytes() const {
	return (size_t)filestream.gcount();
}

//------------------------------------------------------------------------------

FileWriteStream::FileWriteStream(const char *filePath) : FileStream(filePath, std::ios::out | std::ios::binary | std::ios::trunc) {
}

void FileWriteStream::readBytes(void *dst, size_t size) const {
	throw MsgStreamRead(Message::LEVEL_CRIT, "FileWriteStream::readBytes(): Unable to read from \"%s\"", filePath);
}

FileReadStream::FileReadStream(const char *filePath) :	FileStream(filePath, std::ios::in | std::ios::binary) {
}

void FileReadStream::writeBytes(const void *src, size_t size) {
	throw MsgStreamWrite(Message::LEVEL_CRIT, "FileReadStream::writeBytes(): Unable to write to \"%s\"", filePath);
}

//------------------------------------------------------------------------------

