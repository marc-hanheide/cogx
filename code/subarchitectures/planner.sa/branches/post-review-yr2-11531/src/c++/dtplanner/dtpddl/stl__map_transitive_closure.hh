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
#ifndef STL__MAP_TRANSITIVE_CLOSURE_HH
#define STL__MAP_TRANSITIVE_CLOSURE_HH

#include "global.hh"

template<typename T>
std::map<T, std::set<T>>& transitive_closure(std::map<T, std::set<T>>& to_be_closed)
{
    
    std::set<T> domain;
    for(auto p = to_be_closed.begin(); p != to_be_closed.end(); p++) domain.insert(p->first);
    
    bool something_changed = false;
    do{
        for(auto p = domain.begin(); p != domain.end(); p++){
            /*transitive closure implies reflexivity.*/
            to_be_closed[*p].insert(*p);
            
            std::set<T> range = to_be_closed[*p];
            
            for(auto q = range.begin(); q != range.end(); q++){

                assert( to_be_closed[*p].find(*q) != to_be_closed[*p].end());
                    
                std::vector<T> difference;
                auto end_pointer =
                    std::set_difference(to_be_closed[*q].begin(), to_be_closed[*q].end()
                                        , range.begin(), range.end()
                                        , difference.begin());

                for(auto begin_pointer = difference.begin()
                        ; begin_pointer != end_pointer
                        ; begin_pointer++){

                    assert(to_be_closed.find(*p) != to_be_closed.end());
                    
                    to_be_closed[*p].insert(*begin_pointer);
                    if(!something_changed)something_changed = true;
                }
            }
        }
    }while(something_changed);

    return to_be_closed;
}


#endif
