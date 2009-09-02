// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

/*
 *
 * \class{Curses} \member{draw()} function was taken from Norman
 * Matloff's "Introduction to the Unix Curses Library".
 *
 */

#ifndef CURSES_HH
#define CURSES_HH

#include"global.hh"

#include<curses.h>


class Curses
{
public:
    Curses();

    /*Writes the argument to the screen at location \local{nrow} -
     * \local{ncol}.*/
    void draw(char ch);
    
    /* Write the \argument{string} to the top right of the terminal. We
     * assume that the argument string does not contain a newline
     * character, and that the length of the string is shorter than
     * the number of rows in the terminal). A warning is given if our
     * assumptions are not met.*/
    void writeTopRight(const string&);
    
    /* Write the \argument{string} to the bottom left of the terminal. We
     * assume that the argument string does not contain a newline
     * character, and that the length of the string is shorter than
     * the number of rows in the terminal). A warning is given if our
     * assumptions are not met.*/
    void writeBottomLeft(const string&);
    
    /* Write the \argument{string} to the top left of the terminal. We
     * assume that the argument string does not contain a newline
     * character, and that the length of the string is shorter than
     * the number of rows in the terminal). A warning is given if our
     * assumptions are not met.*/
    void writeTopLeft(const string&);
    
    /* Write the \argument{string} to the bottom right of the terminal. We
     * assume that the argument string does not contain a newline
     * character, and that the length of the string is shorter than
     * the number of rows in the terminal). A warning is given if our
     * assumptions are not met.*/
    void writeBottomRight(const string&);

#ifdef WITH_CURSES
    /*We will make a version of this class for each application.*/
    static Curses curses;
#endif
private:
    int row,col, // current row and column (upper-left is (0,0))
	nrows, // number of rows in window
	ncols; // number of columns in window
    
    WINDOW *wnd;
};

#define TOP_LEFT 0
#define TOP_RIGHT 1
#define BOTTOM_LEFT 2
#define BOTTOM_RIGHT 3

#ifdef WITH_CURSES
#define CURSES_WRITE(WHERE, STR) {		\
	if(WHERE == TOP_LEFT)			\
	{					\
	    Curses::curses.writeTopLeft(STR);	\
	} else if (WHERE == TOP_RIGHT) {		\
	    Curses::curses.writeTopRight(STR);		\
	} else if (WHERE == BOTTOM_LEFT) {		 \
	    Curses::curses.writeBottomLeft(STR);	 \
	} else {					 \
	    Curses::curses.writeBottomRight(STR);	 \
	}						 \
    }							 \
	

#define CURSES_OSS_WRITE(WHERE, ARG) { \
	ostringstream oss;	       \
	oss<<ARG;		       \
	CURSES_WRITE(WHERE, oss.str()); \
    }				       \
	

#else

#define CURSES_WRITE(WHERE, STR) {}
#define CURSES_OSS_WRITE(WHERE, ARG){}

#endif

#endif
