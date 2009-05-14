
include ( ../begin.pri )

TEMPLATE = lib
TARGET = Cmd
VERSION = 1.0

#CONFIG -= release
#CONFIG += debug

SOURCES += BoolOption.C \
	CmdOption.C \
	CmdParser.C \
	EnumOption.C \
	StringArgument.C \
	StringOption.C 

include(../end.pri)
