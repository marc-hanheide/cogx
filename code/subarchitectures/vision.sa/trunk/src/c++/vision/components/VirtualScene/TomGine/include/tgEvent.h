//HEADER:
//			Title:			class tgEvent
//			File:				tgEvent.h
//
//			Function:		Describing Input Events
//
//			Author:			Thomas Mörwald
//			Date:				05.01.2009
// ----------------------------------------------------------------------------

#ifndef TG_EVENT
#define TG_EVENT

#include <X11/Xlib.h>

struct tgKey{
	KeySym keysym;
};

struct tgMouseButton{
	unsigned int button;
};

struct tgMouseMotion{
	int x, y;
	int x_rel, y_rel;
};

struct tgExpose{
	int width, height;
};

struct tgClientMessage{
	bool stop;
	tgClientMessage(){
		stop = false;
	}
};

struct tgEvent{
	int type;
	tgKey key;
	tgMouseButton button;
	tgMouseMotion motion;	
	tgExpose expose;
	tgClientMessage clientmessage;
};

#endif