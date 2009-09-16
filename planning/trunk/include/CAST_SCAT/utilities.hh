
/* Copyright (C) 2009
 * Charles Gretton (charles.gretton@gmail.com)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ABOUT ::
 *
 * \library{STL}, \library{TR1}, and \library{boost} includes. Also, a
 * few general pieces of code.
 *
 */

#ifndef UTILITIES_HH
#define UTILITIES_HH

/*---------------------------------

Usual suspects C

---------------------------------*/
#include<cstddef>
#include<cstdio>
#include<cmath>
#include<cstdlib>
//#include<cmalloc>
#include<cassert>
#include<cctype>
#include<csignal>
#include<cstdarg>
#include<cstddef>
#include<cstring>
/*---------------------------------*/

/*---------------------------------

Usual suspects C++::1998

---------------------------------*/
#include <string>
#include <vector>

/* Streaming...*/
#include <iostream>
#include <iomanip> /*setw, etc.*/
#include <sstream> /*string-stream*/



/* (also, see \module{tr1/type_traits} ).*/
#include <cxxabi.h>
/*---------------------------------*/

/*---------------------------------

Usual suspects C++:: c++-0x

---------------------------------*/

/* The kind folks at the GCC team have put all the TR1 and C++-0X
 * stuff in tr1. Some of it is also off-tr1, but I prefer to note
 * where features are not entirely concrete...*/
#include <tr1/tuple>
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <tr1/memory>

/* Functionality for bringing (static -- i.e., compile time) type
 * information into the running system.*/
#include <tr1/type_traits>

/*---------------------------------*/


/*---------------------------------

Usual suspects (C++) BOOST

---------------------------------*/
#include<boost/functional/hash.hpp>
/*---------------------------------*/


/* Sometimes you need a class.. but don't really need a class. */
class NA
{};

#ifndef VECTOR_PRINTING

#define VECTOR_PRINTING

/* Function to printing a \class{vector} to a stream.*/
template<typename T>
std::ostream& operator<<(std::ostream& o, const std::vector<T>& input)
{
    for(auto p = input.begin(); p != input.end(); p++){ 
        o<<*p
         <<((input.end() == p + 1)?"":", ");
    }
    

    return o;
}

#endif

#endif
