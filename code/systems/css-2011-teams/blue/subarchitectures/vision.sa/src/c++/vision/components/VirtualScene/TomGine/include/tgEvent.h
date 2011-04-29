 /**
 * @file tgEvent.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Structure defining keyboard and mouse input events.
 */

#ifndef TG_EVENT
#define TG_EVENT

#include <X11/Xlib.h>

namespace TomGine{

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

} // namespace TomGine

#endif