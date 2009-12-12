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
 */

#ifndef STL__TUPLE_HASH_HH
#define STL__TUPLE_HASH_HH

#include "debug.hh"

/* At the time of writing, the most recent version of GCC did not
 * support \class{std::tuple} hashing. I know the functional folk have
 * their own word for what this template does, but I can't remember
 * it, and can't be bothered looking it up. I am calling it
 * "unwinding". No-one other than me need ever look at this
 * anyway. 
 *
 * To unwind a \class{std::tuple}, is to do the following...
 *
 */
template<typename T, typename Function, int i, typename ...Args>
class unwind_tuple 
{
public:
    typedef unwind_tuple<T, Function, i - 1, Args...> CHILD;
    
    T operator()(const std::tr1::tuple<const Args&...>& args, const Function& function) const {
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
    T operator()(const std::tr1::tuple<const Args&...>& args, const Function& function) const
    {
        T t = 0;
        
        function(t, std::tr1::get<0>(args));
        
        return t;
    }
};

/* Suppose you want to compute a unique hash given an
 * \class{std::tuple} instance. Sure, c++-0x probably already has this
 * feature (officially), and moreover it should. But GCC doesn't at
 * the time of writing. Then if you combine element hashing using
 * \argument{Hash_Combine} to obtain the final hash, the following is
 * useful to you. \argument{Args...} are the types of the
 * \class{std::tuple} arguments, in order...*/
template<typename Hash_Combine, typename ...Args>
class tuple_hasher
{
public:
    std::size_t operator()(const std::tr1::tuple<const Args&...>& args) const
    {
        assert((sizeof...(Args)) - 1);
        unwind_tuple<std::size_t, Hash_Combine, (sizeof...(Args)) - 1, Args...> tmp;
        
        return tmp(args, hash_Combine);
    }
    
private:
    Hash_Combine hash_Combine;
};

/* C++-0x support in GCC is still in the early stages. Partially
 * instantiated template functions are not supported yet. Hopefully
 * one day they will be.  In the mean time, to pass an uninstantiated
 * (parametrically speaking) template function (such as boosts
 * \function{hash_combine}) as an argument to something, you can wrap
 * it as follows.*/
class hash_combine__boost_wrapper 
{
public:
    template<typename T>
    void operator()(std::size_t& size, const T& t) const
    {
        boost::hash_combine(size, t);
    }
    
};

/* \library{boost} based tuple hasisng.*/
template<typename ...Args>
class boost_combine_based_tuple_hasher : public tuple_hasher<hash_combine__boost_wrapper, Args...>
{};


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
 * VYVYAN: Cutting up bodies for my course, you know.
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
