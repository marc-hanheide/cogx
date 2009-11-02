// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#include "Curses.hh"

#ifdef WITH_CURSES
Curses Curses::curses;
#endif

Curses::Curses()
{
    wnd = initscr(); //  initialise window
    
    cbreak(); //  set no waiting for Enter key
    
    noecho(); //  set no echoing
    
    getmaxyx(wnd,nrows,ncols); //  find size of window
    
    clear(); //  clear screen, send cursor to position (0,0)
    
    refresh(); //  implement all changes since last refresh
}

void Curses::draw(char ch)
{
    move(row,col); //  move cursor to row r, column c

    // replace character under cursor by ch
    delch();
    insch(ch); 
    
    refresh(); //  update screen
    
    col++; // go to next row
    
//     // check for need to shift right or wrap around
//     if (row == nrows) {
// 	row = 0;
// 	col++;
// 	if (col == ncols) {
// 	    col = 0;
// 	}
//     }
}

void Curses::writeTopRight(const string& str)
{
    /*Make sure that our assumptions are met.*/
    if(str.find('\n') != str.length()){
	WARNING("\\class{Curses} was given a string with a newline character"<<endl
		<<" ** to write to the terminal. This was no expected."<<endl);
    }
    if(str.length() >= ncols){
	WARNING("\\class{Curses} was given too long a string to write to the screen.\n");
    }

    /*Make sure the cursor is pointing to the right spot on the
     * terminal, given the length of our string.*/
    row = 0;
    col = ncols - str.length() - 1;
    
    
    for(int i = 0; i < str.length(); i++){
	draw(str[i]);
    }

    move(nrows - 1, 0);
    refresh(); //  update screen
}
    
void Curses::writeBottomLeft(const string& str)
{
    /*Make sure that our assumptions are met.*/
    if(str.find('\n') != str.length()){
	WARNING("\\class{Curses} was given a string with a newline character"<<endl
		<<" ** to write to the terminal. This was no expected."<<endl);
    }
    if(str.length() >= ncols){
	WARNING("\\class{Curses} was given too long a string to write to the screen.\n");
    } 

    

    /*Make sure the cursor is pointing to the right spot on the
     * terminal, given the length of our string.*/
    row = nrows - 1;
    col = 0;//ncols - str.length() - 1;
    
    
    for(int i = 0; i < str.length(); i++){
	draw(str[i]);
    }

    move(nrows - 1, 0);
    refresh(); //  update screen
}
    
void Curses::writeTopLeft(const string& str)
{
    /*Make sure that our assumptions are met.*/
    if(str.find('\n') != str.length()){
	WARNING("\\class{Curses} was given a string with a newline character"<<endl
		<<" ** to write to the terminal. This was no expected."<<endl);
    }
    if(str.length() >= ncols){
	WARNING("\\class{Curses} was given too long a string to write to the screen.\n");
    } 

    /*Make sure the cursor is pointing to the right spot on the
     * terminal, given the length of our string.*/
    row = 0;
    col = 0;//ncols - str.length() - 1;
    
    
    for(int i = 0; i < str.length(); i++){
	draw(str[i]);
    }

    move(nrows - 1, 0);
    refresh(); //  update screen
}
    
void Curses::writeBottomRight(const string& str)
{
    /*Make sure that our assumptions are met.*/
    if(str.find('\n') != str.length()){
	WARNING("\\class{Curses} was given a string with a newline character"<<endl
		<<" ** to write to the terminal. This was no expected."<<endl);
    }
    if(str.length() >= ncols){
	WARNING("\\class{Curses} was given too long a string to write to the screen.\n");
    } 
    

    /*Make sure the cursor is pointing to the right spot on the
     * terminal, given the length of our string.*/
    row = nrows - 1;
    col = ncols - str.length() - 1;
    
    
    for(int i = 0; i < str.length(); i++){
	draw(str[i]);
    }

    move(nrows - 1, 0);
    refresh(); //  update screen
}

