/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
 *
 * Authorship of this source code was supported by EC FP7-IST grant
 * 215181-CogX.
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
 * Dear CogX team member :: Please email (charles.gretton@gmail.com)
 * if you make a change to this and commit that change to SVN. In that
 * email, can you please attach the source files you changed as they
 * appeared before you changed them (i.e., fresh out of SVN), and the
 * diff files (*). Alternatively, you could not commit your changes,
 * but rather post me a patch (**) which I will test and then commit
 * if it does not cause any problems under testing.
 *
 * (*) see http://www.gnu.org/software/diffutils/diffutils.html --
 * GNU-09/2009
 *
 * (**) see http://savannah.gnu.org/projects/patch -- GNU-09/2009
 * 
 */


#ifndef STL__TUPLE_TOOLS_HH
#define STL__TUPLE_TOOLS_HH


#include "global.hh"
// #include "debug.hh"


/* At the time of writing, the most recent version of GCC did not
 * support \class{std::tr1::tuple} hashing. I know the functional folk
 * have their own word for what this template does, but I can't
 * remember it, and can't be bothered looking it up. I am calling it
 * "unwinding". No-one other than me need ever look at this anyway.
 *
 * To unwind a \class{std::tr1::tuple}, is to do the following...
 *
 */
template<typename T, typename Function, int i, typename ...Args>
class unwind_tuple 
{
public:
    typedef unwind_tuple<T, Function, i - 1, Args...> CHILD;
    
    T operator()(const std::tr1::tuple<Args...>& args, const Function& function) const {
        CHILD child;

        T t = child(args, function);
        function(t, std::tr1::get<i>(args));
        
        return t;
    }
};

/* \class{unwind_tuple} continued...*/
template<typename T, typename Function,  typename ...Args>
class unwind_tuple<T, Function, 0, Args...>
{
public:
    T operator()(const std::tr1::tuple<Args...>& args, const Function& function) const
    {
        T t = function.default_return_value;//0;
        
        function(t, std::tr1::get<0>(args));
        
        return t;
    }
};




#endif


/*
 * \begin{quote}
 * 
 * ...
 *
 * VYVYAN: I've been down the morgue!
 * 
 * NEIL: Oh, fine, yeah, great. Let's talk about death, I mean, don't
 * consider my feelings tonight, or anything, really.
 * 
 * VYVYAN: Cutting up bodies for my course, you know...
 * 
 * RICK: None of you ever give the slightest consideration to a word I've
 * said!
 * 
 * VYVYAN: That's because you're very boring!
 * 
 * RICK: Oh! Oh, and I suppose you think ideas like peace and freedom and
 * equality are boring too!
 * 
 * VYVYAN: Yes, they are!
 * 
 * RICK: Ha! Fallen into my trap! In that case, why isn't Cliff Richard
 * boring, clever-trousers? Tell me that!
 * 
 *       [VYVYAN responds by pushing RICK's face down into his food.]
 * 
 * NEIL: Okay, lads, umm... this is it, okay, I'm going, now, this is the
 * final moment, okay? Right.
 * 
 * VYVYAN: I've got a leg.
 * 
 * MIKE: Hey Vyvyan, that's not unusual.
 * 
 * VYVYAN: No, look. I'm supposed to write an essay on it, right,
 * but... I think I'm just gonna stick it on the bonnet of my car!
 * 
 * \end{quote}
 * 
 *  -- Ben Elton, Rik Mayall, and Lise Mayer. The Young Ones -- TV
 * Series Premiere "Demolition". Original Airdate 09/11/1982.
 *
 */
