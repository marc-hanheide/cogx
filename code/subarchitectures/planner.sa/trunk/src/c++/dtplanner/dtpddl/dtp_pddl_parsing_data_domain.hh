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
#ifndef DTP_PDDL_PARSING_DATA_DOMAIN_HH
#define DTP_PDDL_PARSING_DATA_DOMAIN_HH

#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_constants.hh"

#include "dtp_pddl_parsing_data_formula.hh"


#include "planning_action_schema.hh"
#include "planning_observation_schema.hh"


namespace Planning
{
    namespace Parsing
    { 
        class Domain_Data : public Constants_Data,
                            public Formula_Data
        {
        public:
            friend std::ostream& std::operator<<(std::ostream&, const Planning::Parsing::Domain_Data&);
            friend class Problem_Data;

//             Domain_Data();

            void stack__typed_Arguments();


            void add__observation_execution();
            void add__observation_precondition();
            void add__observation_effect();
            void add__observation();
            void report__observation_name(const std::string& str);
            void add__observation_header();

            
            
            void add__action_precondition();
            void add__action_effect();
            void add__action();
            void report__action_name(const std::string& str);
            void add__action_header();

            
            void add__domain_Name(const std::string&);
            Planning::Domain_Name get__domain_Name() const ;
            
            enum Function_Range {t_int, t_number, t_double, t_type};
            void set_function_range__number();
            void set_function_range__int();
            void set_function_range__double();
            void set_function_range__type(const std::string& str);

            
            
            /* The range of the last function symbol parsed, were that
             * range was specified during the parsing of the
             * symbol. */
            Function_Range function_Range;

            /* IF (\member{function_Range} == Function_Range::t_type),
             * THEN this member
             * (\member{range_of_last_function_parsed}) will contain
             * the type Specification of the function's range.*/
            Type range_of_last_function_parsed;
            
            
//             void add__number (const std::string& str);
//             void report__parsing_real_number();
//             void report__parsing_integer_number();

            
            void add__requirement(const std::string& str);//{domain_requirements.push_back(str);};
            
//             /* In PDDL we have a specification element
//              * (:types typename_1 typename_2 ... - (either typename_N typename N+1 ... ) )
//              */
//             void add__type(const std::string& str);//{types.push_back(str);};
//             void add__type_of_type(const std::string& str);
//             /* Adds the entries in \member{types} and
//              * \member{types_of_types} to
//              * \member{types_description}. The former are strings
//              * occurring before the '-' is a PDDL type
//              * specification. The latter occur after that in an
//              * "(either T1 T2 ...)" clause.*/
//             void add__types();
//             /* The contents of \member{types} is treated as domain
//              * types from hereon -- unless a further commitment is
//              * made. */
//             void commit__types();

            
            
            void add__derived_predicate_header();
            void commit__derived_predicate();


            
//             /* When parsing a formulae, we keep track of how deep in
//              * the parse tree we are.*/ 
//             void report__dive();
//             void report__emerge();

            
//         private:
//             /*For \argument{quantifier} see \enum{enum_types} from \module{planning_symbols}.*/
//             Formula::Subformula complete__quantified_formula(int quantifier);
//             /*(see \member{complete__quantified_formula(forall)})*/
//             Formula::Subformula complete__forall_formula();
//             /*(see \member{complete__quantified_formula(exists)})*/
//             Formula::Subformula complete__exists_formula();



        public:
            
            
//             void address___derived_Predicates__types_pending();


//             template<typename PROPOSITION_SYMBOL
//                      , typename PREDICATE_SYMBOL
//                      , typename Symbol_Name>
//             void report__formula_atomic_symbol(const Symbol_Name& symbol_name){
//                 VERBOSER(22, "Parsing percept as part of formula.");
    
//                 //     assert(formula_parsing_level - 1 >= 0);
//                 if(subformulae.find(formula_parsing_level) == subformulae.end()){
//                     VERBOSER(22, "Preparing space to add percept at level ::"<<(formula_parsing_level)<<std::endl);
//                     subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
//                     VERBOSER(22, ":: DONE ::"<<std::endl);
//                 }

//                 bool got_proposition = true;
//                 Planning::Constant_Arguments constant_Arguments;
//                 for(auto argument = argument_List.begin()
//                         ; argument != argument_List.end()
//                         ; argument++){
//                     if(argument->test_cast<Variable>()){
//                         got_proposition = false;
//                         auto tmp = argument->do_cast_and_copy<Planning::Variable>();
//                         variables.insert(tmp);
//                     } else if (got_proposition){
//                         constant_Arguments.push_back(argument->do_cast_and_copy<Planning::Constant>());
//                     }
//                 }
    
//                 assert(subformulae.find(formula_parsing_level) != subformulae.end());
    
//                 if(!got_proposition){
//                     NEW_object_referenced_WRAPPED_deref_POINTER
//                         (PREDICATE_SYMBOL
//                          , atomic_symbol
//                          , symbol_name
//                          , argument_List);
//                     subformulae[formula_parsing_level].push_back(atomic_symbol);
//                 } else {
//                     NEW_object_referenced_WRAPPED_deref_POINTER
//                         (PROPOSITION_SYMBOL
//                          , atomic_symbol
//                          , symbol_name
//                          , constant_Arguments);
//                     subformulae[formula_parsing_level].push_back(atomic_symbol);
//                 }

//                 VERBOSER(101, "PARSED :: "<<formula_parsing_level<<":: "<<subformulae[formula_parsing_level].back()<<std::endl);
    
//                 argument_List = Argument_List (); 
//             }
            

//             /*(see \member{report__formula_atomic_symbol})*/
//             void report__formula_predicate();
//             /*(see) \member{report__formula_atomic_symbol}*/
//             void report__formula_percept();
//             /*(see) \member{report__formula_atomic_symbol}*/
//             void report__formula_function();
//             /*(see) \member{report__formula_atomic_symbol}*/
//             void report__formula_action();
            
//             void report__empty_formula();
//             void report__if_formula();
//             void report__not_formula();
//             void report__and_formula();
//             void report__or_formula();
//             void report__exists_formula();
//             void report__forall_formula();
            
//             void report__skip_next____report__formula();
//             void report__formula(const std::string&);

            

            /*Obtains the type of the argument variable from the indicated structure.*/
            Planning::Types find__derived_Predicate_Header(const Planning::Variable&) const;
            Planning::Types find__action_Header(const Planning::Variable&) const;
            Planning::Types find__type_of_variable(const Planning::Variable&) const;
            
            
            
//             void commit__constants();
//             void convert__type_of_type__TO__type_of_constant();
//             void add__constants();
//             void add__constant(const std::string& str);
//             void add__type_of_constant(const std::string& str);

            void add__percept();
            
            void add__predicate();

            const Planning::Action_Schemas& get__action_Schemas() const;
            
        private:
            /*Domain observations.*/
            Planning::Observation_Schemas observation_Schemas;
            
            /*Last observation action precondition.*/
            Planning::Formula::Subformula observation_execution;
            
            /*Last observation precondition formula parsed.*/
            Planning::Formula::Subformula observation_precondition;
            
            /*Last observation effect formula parsed.*/
            Planning::Formula::Subformula observation_effect;
            
            /*Domain actions.*/
            Planning::Action_Schemas action_Schemas;
            
            /*Last action precondition formula parsed.*/
            Formula::Subformula action_precondition;
            
            /*Last action effect formula parsed.*/
            Formula::Subformula action_effect;
            
            /*Parsing the headers of derived predicates.*/
            Planning::Derived_Predicate_Header derived_Predicate_Header;
            
            /* Predicates whose truth value is derived from that of
             * other predicates. */
            Planning::Derived_Predicates derived_Predicates;

            /* Parsed observation name.*/
            Planning::Observation_Name observation_Name;

            /* Set of observation headers parsed.*/
            Planning::Observation_Header observation_Header;
            
            /* Set of action headers parsed.*/
            Planning::Action_Header action_Header;
            
            /* Set of predicates parsed.*/
            Planning::Predicate_Descriptions predicate_Descriptions;
            
            /* Set of predicates parsed.*/
            Planning::Percept_Descriptions percept_Descriptions;
            
            /*String name of \object{*this} domain.*/
            Planning::Domain_Name domain_Name;

            /* PDDL domain requirements (i.e., parsed from PDDL
             * "(:requirements ... )" bit ).*/
            Planning::Requirements domain_requirements;

        public:
            typedef decltype(Domain_Data::domain_Name) Identifier;
        };
    }
}


#endif
