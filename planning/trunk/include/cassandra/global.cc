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
 */

#include "global.hh"

using std::ostringstream;
using std::endl;
using std::string;

bool HasStringRepresentation::operator<(const HasStringRepresentation& hsr) const
{
    /*FIX :: efficienct (i.e., repeat conditional from overloaded
     * \function{std::operator<<} below...).*/
    VERBOSER(5, "Calling LEQ test for HasStringRepresentation.\n");
    
    ostringstream oss;
    oss<<*this;
    ostringstream oss1;
    oss1<<hsr;

    VERBOSER(5, "EQ test between "
	     <<*this<<" and "<<hsr<<" yields :: "<<(oss.str() < oss1.str())<<endl);
    
    return oss.str() < oss1.str();
}

bool HasStringRepresentation::operator==(const HasStringRepresentation& hsr) const
{
    /*FIX :: efficienct (i.e., repeat conditional from overloaded
     * \function{std::operator<<} below...).*/
    VERBOSER(5, "Calling EQ test for HasStringRepresentation.\n");
    ostringstream oss;
    oss<<*this;
    ostringstream oss1;
    oss1<<hsr;

    VERBOSER(5, "EQ test between "
	     <<*this<<" and "<<hsr<<" yields :: "<<(oss.str() == oss1.str())<<endl);
    
    return oss.str() == oss1.str();
}

size_t HasStringRepresentation::hash_value() const
{
    /*FIX :: efficienct (i.e., repeat conditional from overloaded
     * \function{std::operator<<} below...).*/
    ostringstream oss;
    oss<<*this;
    
    return hasher(oss.str());
}


HasStringRepresentation::HasStringRepresentation(const string& str, bool b)
    :asString(str),computedAsString(b)
{}


HasStringRepresentation::HasStringRepresentation(string&& str, bool b)
    :asString(str),computedAsString(b)
{}

HasStringRepresentation::~HasStringRepresentation(){}
    
HasStringRepresentation::HasStringRepresentation(const HasStringRepresentation& hsr)
    :asString(hsr.asString),
     computedAsString(hsr.computedAsString)
{}

HasStringRepresentation::HasStringRepresentation(HasStringRepresentation&& hsr)
    :asString(std::move(hsr.asString)),
     computedAsString(hsr.computedAsString)
{}
    
HasStringRepresentation& HasStringRepresentation::operator=(HasStringRepresentation&& hsr)
{
    asString = std::move(hsr.asString);
    computedAsString = computedAsString;

    return *this;
}
    
HasStringRepresentation& HasStringRepresentation::operator=(const HasStringRepresentation& hsr)
{
    asString = hsr.asString;
    computedAsString = computedAsString;

    return *this;
}


std::size_t hash_value(const HasStringRepresentation& hsr)
{
    return hsr.hash_value();
}

namespace std
{
    std::ostream& operator<<(std::ostream& o,
                             const HasStringRepresentation& hasStringRepresentation)
    {
        if(!hasStringRepresentation.computedAsString){
            hasStringRepresentation.computeAsString("");
        }
        
        return o<<hasStringRepresentation.asString;
    }
}

