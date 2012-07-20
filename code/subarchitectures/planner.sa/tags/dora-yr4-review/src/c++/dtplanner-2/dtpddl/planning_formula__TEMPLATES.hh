/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
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
#ifndef PLANNING_FORMULA__TEMPLATES_HH
#define PLANNING_FORMULA__TEMPLATES_HH


#define PRINTING_SYMBOL_IMPLEMENTATION(TYPENAME)                        \
    template<int ID_VAL, typename NAMING_TYPE>                          \
    ostream& TYPENAME<ID_VAL, NAMING_TYPE>::operator<<(ostream&o) const \
    {                                                                   \
        auto& contents = this->Parent::contents();                      \
        auto& name = std::tr1::get<0>(contents);                        \
        if(PRINTING_WITH_THREAD_INTEGER){                               \
            o<<this->get__runtime_Thread();                             \
        }                                                               \
                                                                        \
        o<<"("<<name<<" ";                                              \
        Printing::operator<<(o, std::tr1::get<1>(contents));            \
        o<<")"<<std::endl;                                              \
        return o;                                                       \
    }                                                                   \


namespace  Planning
{
    namespace Formula
    {
        PRINTING_SYMBOL_IMPLEMENTATION(Predicate)
        PRINTING_SYMBOL_IMPLEMENTATION(Proposition)
    }
}


#endif
