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
#ifndef STL__TUPLE_PRINTING_HH
#define STL__TUPLE_PRINTING_HH

#include "stl__tuple_tools.hh"

namespace std
{    
    template<typename ...T>
    std::ostream& operator<<(std::ostream& o, const std::tr1::tuple<T...>& tuple);
    
}

template<typename Print_Function, typename ...Args>
class tuple_printer
{
public:
    tuple_printer(std::ostream& o)
        :function(o){};
    
    std::ostream& operator()(const std::tr1::tuple<Args...>& args) const
    {
//         assert((sizeof...(Args)) - 1);
//         static_assert((sizeof...(Args)) - 1, "No contents in a printed tuple.");

//         static_assert((sizeof...(Args)) >= 0, ".");
        
        unwind_tuple<std::ostream&, Print_Function, (sizeof...(Args)) - 1, Args...> tmp;
        return tmp(args, function);
    }
    
private:
    Print_Function function;
};

class ostream_print
{
public:
    ostream_print(std::ostream& o):default_return_value(o){};
    
    void operator()(std::ostream& o, int t) const
    {
//         std::operator<<(o,t);
        o<<t;
    }
    
    template<typename T>
    void operator()(std::ostream& o, const T& t) const
    {
        //std::operator<<(o,t);
        o<<t;
    }
    
    std::ostream& default_return_value;
};

template<typename ... Args>
class tuple_printing : public tuple_printer<ostream_print, Args...>{
public:
    tuple_printing(std::ostream&o):tuple_printer<ostream_print, Args...>(o){};
};


namespace std
{    
    template<typename ...T>
    std::ostream& operator<<(std::ostream& o, const std::tr1::tuple<T...>& tuple)
    {
        tuple_printing<T...> tp(o);
        return tp(tuple);
    }
}


#endif
