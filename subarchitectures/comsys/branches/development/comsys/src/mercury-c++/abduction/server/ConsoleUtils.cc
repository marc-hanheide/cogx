#include "ConsoleUtils.h"

extern "C" {
#include <unistd.h>
}

using namespace std;

// term escape sequences
const char * FG_GREEN = "\033[32m";
const char * FG_YELLOW = "\033[33m";
const char * FG_WHITE = "\033[37m";
const char * RESET = "\033[0m";

// ----------------------------------------------------------------

ostream & tty_print(ostream & out, const char * seq)
{
	if (isatty(fileno(stdout)))
		return out << seq;
	else
		return out;
}

// ----------------------------------------------------------------

ostream & tty::green(ostream & out)
{
	return tty_print(out, FG_GREEN);
}

ostream & tty::yellow(ostream & out)
{
	return tty_print(out, FG_YELLOW);
}

ostream & tty::white(ostream & out)
{
	return tty_print(out, FG_WHITE);
}

ostream & tty::dcol(ostream & out)
{
	return tty_print(out, RESET);
}
