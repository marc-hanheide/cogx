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
#include "planning_symbols.hh"

using namespace Planning;

std::ostream& Planning::Predicate_Name::operator<<(std::ostream& o) const
{
    if(PRINTING_WITH_THREAD_INTEGER){
        o<<get__runtime_Thread();
    }
    
    return o<<contents();
}

std::ostream& Planning::Type::operator<<(std::ostream& o) const
{
    if(PRINTING_WITH_THREAD_INTEGER){
        o<<get__runtime_Thread();
    }
    
    return o<<contents();
}

std::ostream& Planning::Constant::operator<<(std::ostream& o) const
{
    if(PRINTING_WITH_THREAD_INTEGER){
        o<<get__runtime_Thread();
    }
    
    return o<<contents();
}

std::ostream& Planning::Variable::operator<<(std::ostream& o) const
{
    if(PRINTING_WITH_THREAD_INTEGER){
        o<<get__runtime_Thread();
    }
    
    return o<<"?"<<contents();
}

std::ostream& Planning::Requirement::operator<<(std::ostream& o) const
{
    return o<<":"<<std::tr1::get<0>(contents());
}


// std::ostream& Planning::Predicate_Description::operator<<(std::ostream& o) const
// {
//     auto name = std::tr1::get<0>(contents());
//     o<<"("<<name<<" ";

//     auto arguments = std::tr1::get<1>(contents());;

//     auto argument_List = std::tr1::get<0>(arguments);
//     auto argument_Types = std::tr1::get<1>(arguments);

//     assert(argument_List.size() == argument_Types.size());
    
    
//     auto argument = argument_List.begin();
//     auto type = argument_Types.begin();
//     for(; argument != argument_List.end(); argument++, type++){
//         o<<*argument<<" - "<<*type<<" ";
//     }
//     o<<")";
    
//     return o;
// }

// std::ostream& Planning::Percept_Description::operator<<(std::ostream& o) const
// {
//     auto name = std::tr1::get<0>(contents());
//     o<<"("<<name<<" ";

//     auto arguments = std::tr1::get<1>(contents());;

//     auto argument_List = std::tr1::get<0>(arguments);
//     auto argument_Types = std::tr1::get<1>(arguments);

//     assert(argument_List.size() == argument_Types.size());
    
    
//     auto argument = argument_List.begin();
//     auto type = argument_Types.begin();
//     for(; argument != argument_List.end(); argument++, type++){
//         o<<*argument<<" - "<<*type<<" ";
//     }
//     o<<")";
    
//     return o;
// }


const Argument_List& Planning::get__symbols(const Typed_Arguments&in)  {return std::tr1::get<0>(in);};
const Argument_Types& Planning::get__types(const Typed_Arguments&in)  {return std::tr1::get<1>(in);};

Argument_List& Planning::get__symbols( Typed_Arguments&in)  {return std::tr1::get<0>(in);};
Argument_Types& Planning::get__types( Typed_Arguments&in)  {return std::tr1::get<1>(in);};

// std::ostream& std::operator<<(ostream&o, const Planning::Argument_List& argument_List)
// {
//     for(auto arg = argument_List.begin()
//             ; arg != argument_List.end()
//             ; arg++){

        
//         const Constant* constant = dynamic_cast<const Constant*>(arg->get());
//         const Variable* variable = dynamic_cast<const Variable*>(arg->get());
        
//         if(variable){
//             o<<*variable;
//         } else if (constant) {
//             o<<*constant;
//         } else {
//             UNRECOVERABLE_ERROR("Unknown argument type.");
//         }
        
//     }
    
//     return o;  
// }


// bool Argument_List::operator==(const Argument_List&) const
// {
// }

// bool Argument_List::operator<(const Argument_List&) const
// {
// }
        
// std::size_t Argument_List::hash_value() const
// {
//     vector<int> tmp;;
//     for(autoarg= begin();arg!= end(); arg++){
//         const Constant* constant = dynamic_cast<const Constant*>(arg->get());
//         const Variable* variable = dynamic_cast<const Variable*>(arg->get());
        
//         if(variable){
//             get__id()
//         } else if (constant) {
            
//         } else {
//             UNRECOVERABLE_ERROR("Unknown argument type.");
//         }
//     }
    
// }


// std::size_t hash_value(const Argument_List& a_list)
// {
//     return a_list.hash_value();
// }


std::ostream& std::operator<<(ostream&o, const Planning::Typed_Arguments& arguments)
{
    auto argument_List = get__symbols(arguments);
    auto argument_Types = get__types(arguments);

    assert(argument_List.size() == argument_Types.size());
    
    auto argument = argument_List.begin();
    auto type = argument_Types.begin();
    for(; argument != argument_List.end(); argument++, type++){
        o<<" "<<*argument<<" - (either ";//<<*type<<" ";

        for(auto type_atom = type->begin()
                ; type_atom != type->end()
                ; type_atom++){
            o<<*type_atom<<" ";
        }

        o<<") ";
    }

    return o;
}

