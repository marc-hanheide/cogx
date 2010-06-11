/* Copyright (C) 2009 Charles Gretton (charles.gretton@gmail.com)
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
 *
 */

#include "cassandra_POMDP__parser.hh"

using namespace POMDP::Parsing;

std::string Indexed_Strings::operator[](uint index) const
{
    assert(index < this->size());
    return const_cast<Named_Numbers&>(named_Numbers)[index];
}

uint Indexed_Strings::operator[](const std::string& str) const
{
    assert(numbered_Names.find(str) != numbered_Names.end());
    return const_cast<Numbered_Names&>(numbered_Names)[str];
}

bool Indexed_Strings::valid(const std::string& str) const
{
    auto iterator = numbered_Names.find(str);

    QUERY_WARNING(iterator == numbered_Names.end(),
                  "For indexed string structure with first element :: "
                  <<((numbered_Names.size())?(numbered_Names.begin()->first):"")
                  <<std::endl
                  <<"We could not find the queried string :: "<<str<<std::endl);
    
    return (iterator != numbered_Names.end());
}

bool Indexed_Strings::valid(uint index) const
{
    
    QUERY_WARNING(index >= named_Numbers.size(),
                  "For indexed string structure with first element :: "
                  <<((named_Numbers.size())?(*named_Numbers.begin()):"")
                  <<std::endl
                  <<"We could not find the queried element at index :: "<<index<<std::endl);
    
    
    return (index < named_Numbers.size());
}

Indexed_Strings::Indexed_Strings()
    :empty_string(""){}

Indexed_Strings::
Indexed_Strings(const Indexed_Strings& indexed_Strings)
    :empty_string(indexed_Strings.empty_string)
{
    for(auto _string = indexed_Strings.named_Numbers.begin()
            ; _string != indexed_Strings.named_Numbers.end()
            ; _string++){
        push_back(*_string);
    }
}


void Indexed_Strings::push_back(const std::string& str)
{
    named_Numbers.push_back(str);
    numbered_Names[str] = named_Numbers.size() - 1;
}

uint Indexed_Strings::get(const std::string& str) const
{
    auto iterator = numbered_Names.find(str);
    
    if(valid(str)){
        return iterator->second;
    } else {
        return std::numeric_limits<uint>::max();
    }
}

const std::string& Indexed_Strings::get(uint index) const
{
    if(valid(index)){
        return named_Numbers[index];
    } else {
        return empty_string;
    }
}
/******************************************************************************************************
 ******************************************************************************************************
 * OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- 
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************/
            
std::ostream& POMDP::Parsing::operator<<(std::ostream& o,
                         const POMDP::Parsing::Indexed_Strings& indexed_Strings)
{
    for(auto i = 0; i < indexed_Strings.named_Numbers.size(); i++){
        o<<indexed_Strings.named_Numbers[i];

        if(i == indexed_Strings.named_Numbers.size() - 1){
        } else {
            o<<" ";
        }
    }
    
    return o;
}
