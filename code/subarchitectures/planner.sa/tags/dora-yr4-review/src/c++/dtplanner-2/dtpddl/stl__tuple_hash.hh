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
#ifndef STL__TUPLE_HASH_HH
#define STL__TUPLE_HASH_HH


#include "stl__tuple_tools.hh"

namespace boost
{
    template<typename ...Args>
    void hash_combine(std::size_t&, const std::tr1::tuple<Args...>&);

    template<typename ...Args>
    std::size_t hash_value(const std::tr1::tuple<Args...>&);
}

/* Suppose you want to compute a unique hash given an
 * \class{std::tr1::tuple} instance. Sure, c++-0x probably already has this
 * feature (officially), and moreover it should. But GCC doesn't at
 * the time of writing. Then if you combine element hashing using
 * \argument{Hash_Combine} to obtain the final hash, the following is
 * useful to you. \argument{Args...} are the types of the
 * \class{std::tr1::tuple} arguments, in order...*/
template<typename Hash_Function, typename ...Args>
class tuple_hasher
{
public:
    std::size_t operator()(const std::tr1::tuple<Args...>& args) const
    {
//         assert((sizeof...(Args)) - 1);
        unwind_tuple<std::size_t, Hash_Function, (sizeof...(Args)) - 1, Args...> tmp;
        
        return tmp(args, function);
    }
    
private:
    Hash_Function function;
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
    hash_combine__boost_wrapper()
        :default_return_value(0){};
    
    template<typename T>
    void operator()(std::size_t& size, const T& t) const
    {
        boost::hash_combine(size, t);
    }
    
    size_t default_return_value;
};


/* \library{boost} based tuple hasisng.*/
template<typename ...Args>
class boost_combine_based_tuple_hasher : public tuple_hasher<hash_combine__boost_wrapper, Args...>
{};

template<typename ...Args>
void boost::hash_combine(std::size_t& in_seed, const std::tr1::tuple<Args...>& in)
{
    std::size_t seed = boost::hash_value(in);
    boost::hash_combine(in_seed, seed);
}
    
template<typename ...Args>
std::size_t boost::hash_value(const std::tr1::tuple<Args...>& in)
{
    boost_combine_based_tuple_hasher<Args...> tuple_hasher;

    std::size_t tmp = tuple_hasher(in);
       
#ifndef NDEBUG 
#ifdef  DEBUG_LEVEL
#if DEBUG_LEVEL < 10000
    /*GARDED*/std::cerr<<"HASH :: "<<tmp<<" for TUPLE :: "<<in<<std::endl;
#endif 
#endif 
#endif
    
    return tmp;
}


#endif
