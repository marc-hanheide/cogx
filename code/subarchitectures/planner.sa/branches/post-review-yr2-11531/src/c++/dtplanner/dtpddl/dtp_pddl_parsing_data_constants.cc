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
#include "dtp_pddl_parsing_data_constants.hh"

using namespace Planning::Parsing;

void Constants_Data::commit__constants()
{
    WARNING(" :: Committing constants(/objects) dtp_parser functionality is redundant :: ");
}

void Constants_Data::convert__type_of_type__TO__type_of_constant()
{    
    QUERY_WARNING(constants.size(),"Described the type of a constant symbol :: "
                  <<*constants.begin()<<std::endl
                  <<"As being of an \"(either T1 T2 ...)\" type. That is bogus"<<std::endl
                  <<"unless there is only one type string occurring in the \"either\" clause.");
    
    types_of_constants.insert(types_of_types.begin(), types_of_types.end());
    types_of_types = Types();
}

void Constants_Data::add__constants()
{  
    for(auto constant = constants.begin()
            ; constant != constants.end()
            ; constant++){
        if(constants_Description.find(*constant) == constants_Description.end()){
            constants_Description[*constant] = Types();
        }

        if(types_of_constants.size() == 0){/*If the constant was given without a type.*/
            
            if(symbol_theory){
                NEW_referenced_WRAPPED(symbol_theory, Planning::Type, object_type, "object");
                constants_Description[*constant].insert(object_type);
            } else {    
                NEW_object_referenced_WRAPPED(Planning::Type, object_type, "object");
                constants_Description[*constant].insert(object_type);
            }
        } else {
            constants_Description[*constant]
                .insert(types_of_constants.begin()
                        , types_of_constants.end());
        }
    }
    
    types_of_constants = Types();
    constants = Constants();
}

void Constants_Data::add__constant(const std::string& str){

    INTERACTIVE_VERBOSER(true, 3101, "Constants pointer is "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(this)<<std::endl);
    NEW_object_referenced_WRAPPED(Planning::Constant, constant, str);
    constants.insert(constant);
}

void Constants_Data::add__type_of_constant(const std::string& str)
{
    if(symbol_theory){
        VERBOSER(3101, reinterpret_cast<basic_type::Runtime_Thread>(this)<<" type :: "<<str<<" is allocated to a theory :: "<<reinterpret_cast<basic_type::Runtime_Thread>(symbol_theory)<<std::endl);
//         {char ch; std::cin>>ch;}
        
        NEW_referenced_WRAPPED(symbol_theory, Type, type, str);
        types_of_constants.insert(type);
    } else {
        VERBOSER(3101, reinterpret_cast<basic_type::Runtime_Thread>(this)<<" type :: "<<str<<" is not allocated to a theory :: "<<reinterpret_cast<basic_type::Runtime_Thread>(this)<<std::endl);
//         {char ch; std::cin>>ch;}
        
        NEW_object_referenced_WRAPPED(Type, type, str);
        types_of_constants.insert(type);
    }
    
}



// const Planning::Constants& Constants_Data::get__constants() const
// {return constants;}

// Planning::Constants Constants_Data::get__constants()
// {return constants;}

const Planning::Types& Constants_Data::get__constantx_types(const Constant& constant) const
{
    
    QUERY_UNRECOVERABLE_ERROR(
        constants_Description.find(constant) == constants_Description.end()
        , "Could not find types for constant :: "<<constant);

    return constants_Description.find(constant)->second;
    
}

Planning::Types Constants_Data::get__constantx_types(const Constant& constant)
{
    QUERY_UNRECOVERABLE_ERROR(
        constants_Description.find(constant) == constants_Description.end()
        , "Could not find types for constant :: "<<constant);

    return constants_Description.find(constant)->second;//constants_Description[constant];
}

const Planning::Constants_Description&
Constants_Data::get__constants_Description() const
{
    return constants_Description;
}

Planning::Constants_Description&
Constants_Data::get__constants_Description()
{
    return constants_Description;
}

            
