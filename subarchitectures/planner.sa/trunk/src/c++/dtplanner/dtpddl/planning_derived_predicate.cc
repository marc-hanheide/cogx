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
#include "planning_derived_predicate.hh"


using namespace Planning;
using namespace Planning::Formula;
using namespace std;


// Planning::Predicate_Name Derived_Predicate_Header::get__name() const
// {return std::tr1::get<0>(this->contents());
// }

// Planning::Typed_Arguments Derived_Predicate_Header::get__arguments() const 
// {return std::tr1::get<1>(this->contents());
// }






// Planning::Predicate_Name
//     Derived_Predicate::get__name() const {return std::tr1::get<0>(this->contents());}
// Planning::Typed_Arguments
//     Derived_Predicate::get__arguments() const {return std::tr1::get<1>(this->contents());}
// Planning::Typed_Arguments
//     Derived_Predicate::get__quantified_symbols() const {return std::tr1::get<2>(this->contents());}
// Planning::Variables
//     Derived_Predicate::get__variables() const {return std::tr1::get<3>(this->contents());}
// CXX__deref__shared_ptr<basic_type>
//     Derived_Predicate::get__formula() const {return std::tr1::get<4>(this->contents());}
// int Derived_Predicate::get__type() const {return std::tr1::get<5>(this->contents());}
// // int Derived_Predicate::get__level() const  {return std::tr1::get<5>(this->contents());}
// // Planning::Variable Derived_Predicate::get__variable() const {return std::tr1::get<5>(this->contents());}
// // int Derived_Predicate::get__genesis() const  {return std::tr1::get<5>(this->contents());}

// std::string Derived_Predicate::get__description() const
// {         
//     ostringstream oss;
    
//     oss<<"(:derived ";

//     oss<<"("<<get__name()<<" ";

//     auto variables = get__symbols(get__arguments());
//     auto variables_types = get__types(get__arguments());

//     assert(variables.size() == variables_types.size());
//     for(auto i = 0; i < variables.size(); i++){

//         oss<<variables[i]<<" - (either ";

//         for(auto type = variables_types[i].begin()
//                 ; type != variables_types[i].end()
//                 ; type++){
//             oss<<*type<<" ";
//         }
        
//         oss<<")  ";
//     }
    
    
//     oss<<" )"<<std::endl;

//     switch(get__type()){
//         case Planning::enum_types::forall :
//         {
//             oss<<"(forall (";

            
//             auto variables = get__symbols(get__quantified_symbols());
//             auto variables_types = get__types(get__quantified_symbols());

//             assert(variables.size() == variables_types.size());
//             for(auto i = 0; i < variables.size(); i++){

//                 oss<<variables[i]<<" - (either ";

//                 for(auto type = variables_types[i].begin()
//                         ; type != variables_types[i].end()
//                         ; type++){
//                     oss<<*type<<" ";
//                 }
        
//                 oss<<")  ";
//             }
            
//             oss<<")"<<std::endl;
//         }
//             break;
//         case Planning::enum_types::exists :
//         {
//             oss<<"(exists (";

            
//             auto variables = get__symbols(get__quantified_symbols());
//             auto variables_types = get__types(get__quantified_symbols());

//             assert(variables.size() == variables_types.size());
//             for(auto i = 0; i < variables.size(); i++){

//                 oss<<variables[i]<<" - (either ";

//                 for(auto type = variables_types[i].begin()
//                         ; type != variables_types[i].end()
//                         ; type++){
//                     oss<<*type<<" ";
//                 }
        
//                 oss<<")  ";
//             }
            
//             oss<<")"<<std::endl;
//         }
//             break;
//     }
    
    
    
//     oss<<get__formula()<<std::endl;

    
//     switch(get__type()){
//         case Planning::enum_types::forall :
//         {
//             oss<<")";
//         }
//             break;
//         case Planning::enum_types::exists :
//         {
//             oss<<")";
//         }
//             break;
//     }
    
    
    
//     oss<<")"<<std::endl;

//     return oss.str();
// }


// ostream& Derived_Predicate::operator<<(ostream&o) const           
// {
//     QUERY_UNRECOVERABLE_ERROR(
//         !is_configured(),
//         "TRYING TO PRINT UNCONFIGURED PREDICATE."<<std::endl);
    
//     auto arguments = get__arguments();
//     auto untyped_arguments = std::tr1::get<0>(arguments);
    
//     o<<"(DERIVED_SYMBOL"<<get__id()<<" ";
//     Planning::Formula::Printing::operator<<(o, untyped_arguments);
//     o<<" )"<<std::endl;
    
//     return o;                                            
// }                                                       
    

// ostream& Derived_Predicate_Header::operator<<(ostream&o) const           
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
    
