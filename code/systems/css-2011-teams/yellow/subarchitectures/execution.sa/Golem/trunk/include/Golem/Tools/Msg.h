/** @file Msg.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_MSG_H_
#define _GOLEM_TOOLS_MSG_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class MsgMessageStream : public Message {MESSAGE_BODY(MsgMessageStream)};
class MsgMessageStreamInvalidDesc : public MsgMessageStream {MESSAGE_BODY(MsgMessageStreamInvalidDesc)};
class MsgMessageStreamThreadLaunch : public MsgMessageStream {MESSAGE_BODY(MsgMessageStreamThreadLaunch)};

class MsgStream : public Message {MESSAGE_BODY(MsgStream)};
class MsgStreamRead : public MsgStream {MESSAGE_BODY(MsgStreamRead)};
class MsgStreamWrite : public MsgStream {MESSAGE_BODY(MsgStreamWrite)};
class MsgStreamFileOpenFail : public MsgStream {MESSAGE_BODY(MsgStreamFileOpenFail)};
class MsgStreamDirCreateFail : public MsgStream {MESSAGE_BODY(MsgStreamDirCreateFail)};

class MsgParallels : public Message {MESSAGE_BODY(MsgParallels)};
class MsgParallelsThreadLaunch : public MsgParallels {MESSAGE_BODY(MsgParallelsThreadLaunch)};

class MsgContext : public Message {MESSAGE_BODY(MsgContext)};
class MsgContextInvalidDesc : public MsgContext {MESSAGE_BODY(MsgContextInvalidDesc)};

class MsgXMLParser : public Message {MESSAGE_BODY(MsgXMLParser)};
class MsgXMLParserInvalidDesc : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserInvalidDesc)};
class MsgXMLParserBufferAlloc : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserBufferAlloc)};
class MsgXMLParserStatusErr : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserStatusErr)};
class MsgXMLParserStatusSuspendErr : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserStatusSuspendErr)};
class MsgXMLParserNullContext : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserNullContext)};
class MsgXMLParserInvalidName : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserInvalidName)};
class MsgXMLParserInvalidCast : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserInvalidCast)};
class MsgXMLParserNameNotFound : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserNameNotFound)};
class MsgXMLParserIncompleteData : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserIncompleteData)};
class MsgXMLParserAttributeNotFound : public MsgXMLParser {MESSAGE_BODY(MsgXMLParserAttributeNotFound)};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_TOOLS_MSG_H_*/
