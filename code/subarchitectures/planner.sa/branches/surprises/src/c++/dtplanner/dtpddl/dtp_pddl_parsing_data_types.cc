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
#include "dtp_pddl_parsing_data_types.hh"

#include "stl__map_transitive_closure.hh"
#include "stl__anti_reflexive.hh"


using namespace Planning::Parsing;

Types_Data::Types_Data()
    :symbol_theory(0)
{
}

Types_Data::~Types_Data(){}


void Types_Data::report__symbol_name_reference(void*symbol_t)
{
   symbol_theory = symbol_t;
}


void Types_Data::commit__types()
{
    VERBOSER(31001, "Unwinding type hirarchy."<<std::endl);
//     {char ch; std::cin>>ch;}
    
    
    Planning::Type object_type;
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory,
                               /*type name*/Planning::Type,
                               /*object name*/_object_type,
                               /*argument to "type name" constructor*/"object");
        object_type = _object_type;
        assert(object_type.get__runtime_Thread() == _object_type.get__runtime_Thread());
    } else {    
        NEW_object_referenced_WRAPPED(/*type name*/Planning::Type,
                                      /*object name*/_object_type,
                                      /*argument to "type name" constructor*/"object");
        object_type = _object_type;
        assert(object_type.get__runtime_Thread() == _object_type.get__runtime_Thread());
    }


    
    /*Is there a PDDL-type called "object"?*/
    if(types_description.find(object_type) == types_description.end()){
        /*If not, make one.*/
        types_description[object_type] = Types();
    }
    
    /* Everything is of PDDL-type "object".*/
    for(auto p = types_description.begin(); p != types_description.end(); p++){
        p->second.insert(object_type);
        assert(p->first.get__runtime_Thread() == object_type.get__runtime_Thread());
    }

    /* A PDDL-type can be of another PDDL-type. Here we obtain an
     * explicit representation of the transitive closure of the
     * PDDL-subtype relation.*/
    transitive_closure<>(types_description);

#ifndef NDEBUG 
    for(auto thing = types_description.begin()
            ; thing != types_description.end()
            ; thing++){
       auto runtt = thing->first.get__runtime_Thread();

       for(auto _thing = thing->second.begin()
               ; _thing != thing->second.end()
               ; _thing++){
           VERBOSER(3101, "Type coherence... "<<runtt<<" -> "<<_thing->get__runtime_Thread());
           assert(runtt == _thing->get__runtime_Thread());
       }
    }
#endif
    
    /* (for the result from the above computation) We don't want loops
     * in the type hierarchy, so we shall remove reflexivity.*/
    anti_reflexive<>(types_description);
    
#ifndef NDEBUG 
    for(auto thing = types_description.begin()
            ; thing != types_description.end()
            ; thing++){
       auto runtt = thing->first.get__runtime_Thread();

       for(auto _thing = thing->second.begin()
               ; _thing != thing->second.end()
               ; _thing++){
           VERBOSER(3101, "Type coherence... "<<runtt<<" -> "<<_thing->get__runtime_Thread());
           assert(runtt == _thing->get__runtime_Thread());
       }
    }
#endif
    
}

Planning::Types Types_Data::find__type_of_variable(const Planning::Variable& in) const
{
    auto answer = find__stack_of__Typed_Arguments(in);
    
    if(answer.size()) return answer;

    UNRECOVERABLE_ERROR("Could not find type for variable :: "<<in<<std::endl);
}


Planning::Types Types_Data::find__stack_of__Typed_Arguments(const Planning::Variable& in) const
{
    std::stack<Typed_Arguments> copy__stack_of__Typed_Arguments
        = stack_of__Typed_Arguments;
    
    while(!copy__stack_of__Typed_Arguments.empty()){
        auto ta = copy__stack_of__Typed_Arguments.top();
        auto variables = Planning::get__symbols(ta);
        auto types = Planning::get__types(ta);

        
        for(uint i = 0
                ; i < variables.size()
                ; i++){
            auto var = variables[i];
            
            assert(var.test_cast<Variable>());
            
            if(in == var.do_cast_and_copy<Variable>()){
                return types[i];
            }
        }
        
        copy__stack_of__Typed_Arguments.pop();
    }
    
    return Types();
}


void Types_Data::add__types(){

    if(types_of_types.size() > 1){
        WARNING(" using the \"Ta Tb ... - (:either T1 T2 ...)\" bit of PDDL to"<<std::endl
                <<"to describe the type hierarchy. I resolve this to:"<<std::endl
                <<"T1 - Ta T2 - Ta.. T1 -Tb...")
        
            for(auto d_type = types_of_types.begin()
                    ; d_type != types_of_types.end()
                    ; d_type++){
            
                if(types_description.find(*d_type) == types_description.end()){
                    types_description[*d_type] = Types();
                }

                for(auto r_type = types.begin()
                        ; r_type != types.end()
                        ; r_type++){
                
                    types_description[*d_type].insert(*r_type);
                }
            }
    } else {
        /*d_ is for domain_*/
        for(auto d_type = types.begin()
                ; d_type != types.end()
                ; d_type++){

            if(types_description.find(*d_type) == types_description.end()){
                types_description[*d_type] = Types();
            }
            
            /*r_ is for range_*/
            for(auto r_type = types_of_types.begin()
                    ; r_type != types_of_types.end()
                    ; r_type++){
            
            
                types_description[*d_type].insert(*r_type);
            }   
        }
    }

    types = decltype(types)();
    types_of_types = decltype(types_of_types)();
    
}

void Types_Data::add__type(const std::string& str){
    if(symbol_theory){
        VERBOSER(3101, reinterpret_cast<basic_type::Runtime_Thread>(this)<<" type hirarchy type :: "<<str<<" is allocated to a theory :: "<<symbol_theory<<" "<<reinterpret_cast<basic_type::Runtime_Thread>(symbol_theory)<<std::endl);
//         {char ch; std::cin>>ch;}
        NEW_referenced_WRAPPED(symbol_theory, Planning::Type, type, str);
        types.insert(type);
    } else {    
        VERBOSER(3101, reinterpret_cast<basic_type::Runtime_Thread>(this)<<" type hirarchy type :: "<<str<<" is not allocated to a theory :: "<<reinterpret_cast<basic_type::Runtime_Thread>(this)<<std::endl);
//         {char ch; std::cin>>ch;}
        NEW_object_referenced_WRAPPED(Planning::Type, type, str);
        types.insert(type);
    }    
}

void Types_Data::add__type_of_type(const std::string& str){
    
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory, Planning::Type, type, str);
        types_of_types.insert(type);
    } else {
        NEW_object_referenced_WRAPPED(Planning::Type, type, str);
        types_of_types.insert(type);
    } 
}


const std::map<Planning::Type, Planning::Types >& Types_Data::get__types_description() const
{
    return types_description;
}

