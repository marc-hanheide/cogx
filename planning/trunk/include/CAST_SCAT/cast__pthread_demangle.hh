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
 * For some reason c++-0x thread initialisation is broken when I
 * compile with CAST. I could not figure out what was causing the
 * breakage (didn't want to look, didn't want to care), so in my
 * frustration I made \macro{A_CAST_SPECIAL__THREAD_INITIALISATION}
 * and related inline factory-function
 * \function{CAST_SCAT::give_me_a_new__pthread_mutex_}.
 * 
 *
 * NOTES :: 
 * 
 * \begin{conversation -- Aug 23 2009}
 *
 * Charles Gretton ::
 *
 * For some reason the macro PTHREAD_MUTEX_INITIALIZER is not working
 * when I add the following code to the tutorial.
 * 
 * ** #include <tr1/memory>
 * ** 
 * **  shared_ptr<pthread_mutex_t> _mutex(new pthread_mutex_t
 * **  PTHREAD_MUTEX_INITIALIZER);
 * 
 * and add -std=c++0x to "advanced" options' CMAKE_CXX_FLAGS
 *
 * 
 * Nick Hawes :: "No idea I'm afraid. Ice has its own pthread wrappers
 * which I've been using in CAST (IceUtil/Thread.h)".
 *
 * \end{conversation}
 *
 */


#ifndef CAST__PTHREAD_DEMANGLE_HH
#define CAST__PTHREAD_DEMANGLE_HH

#include <pthread.h>

#define A_CAST_SPECIAL__THREAD_INITIALISATION(X) pthread_mutex_t* X; \
    {                                                                \
        pthread_mutex_t a_rose_by_any_other_name                     \
            = PTHREAD_MUTEX_INITIALIZER;                             \
        X = new pthread_mutex_t;                                     \
        *X = a_rose_by_any_other_name;                               \
    }                                                                \
    
namespace CAST_SCAT
{
    
    /* FIX :: (see A_CAST_SPECIAL__THREAD_INITIALISATION)...*/
    inline pthread_mutex_t* give_me_a_new__pthread_mutex_t()
    {
        A_CAST_SPECIAL__THREAD_INITIALISATION(X);
        
        QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_init(X, NULL),
                                  "Failed to initialise the mutex...");
        
        return X;
    }
}


#endif
