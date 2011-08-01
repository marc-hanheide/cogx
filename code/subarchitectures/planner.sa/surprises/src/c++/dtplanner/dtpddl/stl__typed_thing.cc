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
#include "stl__typed_thing.hh"

using std::ostream;


void basic_type::visit(Visitor<basic_type>& visitor) const
{
    visitor.c_pointer__accept(this);
}

void basic_type::visit(Visitor<basic_type>& visitor,
                       const CXX__PTR_ANNOTATION(basic_type)& in) const
{
    QUERY_WARNING(in.get() != this,
                  "Unexpected use of visitor interface.");
    visitor.cxx_pointer__accept(in);
}

void basic_type::visit(Visitor<basic_type>& visitor,
                       const CXX__deref__shared_ptr<basic_type>&in) const
{
    QUERY_WARNING(in.get() != this,
                  "Unexpected use of visitor interface.");
    visitor.cxx_deref_pointer__accept(in);
}

void basic_type::visit(Visitor<basic_type>& visitor,
                       const CXX__deref__shared_ptr__visitable<basic_type>&in) const
{
    QUERY_WARNING(in.get() != this,
                  "Unexpected use of visitor interface.");
    
    INTERACTIVE_VERBOSER(true, 5000, "Visiting :: "<<*this<<" with "<<in<<std::endl)
    
    visitor.cxx_deref_pointer_visitable__accept(in);
}


// void basic_type::visit(Visitor<basic_type>& visitor, CXX__PTR_ANNOTATION(basic_type)& input) const
// {
//     QUERY_WARNING(input.get() != this,
//                   "Unexpected use of visitor interface.");

//     visitor.cxx_pointer__accept(input);
// }

// void basic_type::visit(Visitor<basic_type>& visitor, CXX__deref__shared_ptr<basic_type>& input) const
// {
//     QUERY_WARNING(input.get() != this,
//                   "Unexpected use of visitor interface.");
    
//     visitor.cxx_deref_pointer__accept(input);
// }

// void basic_type::visit(Visitor<basic_type>& visitor, CXX__deref__shared_ptr__visitable<basic_type>& input) const
// {
//     QUERY_WARNING(input.get() != this,
//                   "Unexpected use of visitor interface.");
    
//     visitor.cxx_deref_pointer_visitable__accept(input);
// }



basic_type::Runtime_Thread basic_type::get__runtime_Thread() const
{
    return runtime_Thread;
}


std::ostream& basic_type::operator<<(std::ostream&o) const {return o<<hash_value();};

std::string basic_type::as_string() const
{
    std::string str;
    {
        std::ostringstream oss;
        this->operator<<(oss);
        str = oss.str();
    }

    return std::move(str);
}

bool basic_type::operator<(const basic_type&in) const
{return as_string() < in.as_string();};//hash_value() < in.hash_value();};

bool basic_type::operator==(const basic_type&in) const
{ return as_string() == in.as_string();};//hash_value() == in.hash_value();};

// bool basic_type::operator<(const basic_type&in) const
// {return hash_value() < in.hash_value();};

// bool basic_type::operator==(const basic_type&in) const
// { return hash_value() == in.hash_value();};


std::ostream& std::operator<<(std::ostream& o, const basic_type& bt)
{
    return bt.operator<<(o);
}

std::size_t hash_value(const basic_type& bt)
{
    return bt.hash_value();
}
