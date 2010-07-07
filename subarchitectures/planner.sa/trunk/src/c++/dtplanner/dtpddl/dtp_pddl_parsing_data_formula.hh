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
#ifndef DTP_PDDL_PARSING_DATA_FORMULA_HH
#define DTP_PDDL_PARSING_DATA_FORMULA_HH

#include "global.hh"
#include "planning_symbols.hh"

#include "dtp_pddl_parsing_data_types.hh"
#include "planning_derived_predicate.hh"

namespace Planning
{
    namespace Parsing
    {
        class Formula_Data : virtual public Types_Data
        {
        public:

            /* Formulae that are subsequently parsed have as their
             * subject the initial state.  Some formula describe
             * actions pre- and post-conditions, others describe
             * derived predicates. Depending on the context in which a
             * formula is being parsed the operators have a different
             * interpretation. For example, in PDDL the '=' operator
             * in the initial state has the semantics of the 'assign'
             * operator in an action effect. On the other hand, in a
             * 'precondition' the '=' operator has the semantics of an
             * equality test.  */
            void report__enter_parsing_initial_state();
            /* Formulae that are subsequently parsed DO NOT have as
             * their subject the initial state (see
             * \member{report__enter_parsing_initial_state}).*/
            void report__exit_parsing_initial_state();

            void stack__typed_Arguments();
            
            void report__perceptual_function_name(const std::string& str);
            void report__percept_name(const std::string& str);
            void report__state_function_name(const std::string& str);
            void report__predicate_name(const std::string& str);
            
            Formula_Data();
            
            template<typename PROPOSITION_SYMBOL
                     , typename PREDICATE_SYMBOL
                     , typename Symbol_Name>
            void report__formula_atomic_symbol(const Symbol_Name& symbol_name){
                VERBOSER(22, "Parsing percept as part of formula.");
    
                //     assert(formula_parsing_level - 1 >= 0);
                if(subformulae.find(formula_parsing_level) == subformulae.end()){
                    VERBOSER(22, "Preparing space to add percept at level ::"<<(formula_parsing_level)<<std::endl);
                    subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
                    VERBOSER(22, ":: DONE ::"<<std::endl);
                }

                bool got_proposition = true;
                Planning::Constant_Arguments constant_Arguments;
                for(auto argument = argument_List.begin()
                        ; argument != argument_List.end()
                        ; argument++){
                    if(argument->test_cast<Variable>()){
                        got_proposition = false;
                        auto tmp = argument->do_cast_and_copy<Planning::Variable>();
                        variables.insert(tmp);
                    } else if (got_proposition){
                        constant_Arguments.push_back(argument->do_cast_and_copy<Planning::Constant>());
                    }
                }
    
                assert(subformulae.find(formula_parsing_level) != subformulae.end());
    
                if(!got_proposition){
                    NEW_object_referenced_WRAPPED_deref_POINTER
                        (PREDICATE_SYMBOL
                         , atomic_symbol
                         , symbol_name
                         , argument_List);
                    subformulae[formula_parsing_level].push_back(atomic_symbol);
                } else {
                    NEW_object_referenced_WRAPPED_deref_POINTER
                        (PROPOSITION_SYMBOL
                         , atomic_symbol
                         , symbol_name
                         , constant_Arguments);
                    subformulae[formula_parsing_level].push_back(atomic_symbol);
                }

                VERBOSER(101, "PARSED :: "<<formula_parsing_level<<":: "<<subformulae[formula_parsing_level].back()<<std::endl);
    
                argument_List = Argument_List (); 
            }
            

            /*(see \member{report__formula_atomic_symbol})*/
            void report__formula_predicate();
            /*(see) \member{report__formula_atomic_symbol}*/
            void report__formula_percept();
            /*(see) \member{report__formula_atomic_symbol}*/
            void report__formula_perceptual_function();
            /*(see) \member{report__formula_atomic_symbol}*/
            void report__formula_state_function();
            /*(see) \member{report__formula_atomic_symbol}*/
            void report__formula_action();
            
            void report__conditional_effect_formula();
            void report__empty_formula();
            void report__if_formula();
            void report__not_formula();
            void report__and_formula();
            void report__or_formula();
            void report__exists_formula();
            void report__forall_formula();
            
            void report__skip_next____report__formula();
            void report__formula(const std::string&);

            /* When parsing a formulae, we keep track of how deep in
             * the parse tree we are.*/ 
            void report__dive();
            void report__emerge();

            void add__variable_argument(const std::string& str);
            void add__constant_argument(const std::string& str);
            void commit__argument_types();
            
            void add__number (const std::string& str);
            void report__parsing_real_number();
            void report__parsing_integer_number();

            /* There is a function assignment to a constant number in an effect formula.*/
            void report__number_in_formula();
            /* There is a function assignment to a variable-symbol in an effect formula.*/
            void report__object_in_formula();
            /* There is a function assignment to a PDDL constant in an effect formula.*/
            void report__constant_in_formula();
            
            void report__probabilistic_formula();
            void report__increase_formula();
            void report__decrease_formula();
            void report__assign_formula();
            void report__equality_formula();
            Formula::Subformula complete__probabilistic_formula();
            
        private:

            /* Does \member{subformulae} have elements at \argument{index}.*/
            bool check__exists_parsed_subformulae(int index) const;
            
            /* Are there \argument{count} elements at
             * \member{subformulae} element at \argument{index}.*/
            bool check__cardinality_constraint_on_subformulae_at_index
            (int count, int index) const;
            
            /* Does the formula being based have as its subject the
             * initial planning state?*/
            bool parsing_initial_state;
            
            /*For \argument{quantifier} see \enum{enum_types} from \module{planning_symbols}.*/
            Formula::Subformula complete__quantified_formula(int quantifier);
            /*(see \member{complete__quantified_formula(forall)})*/
            Formula::Subformula complete__forall_formula();
            /*(see \member{complete__quantified_formula(exists)})*/
            Formula::Subformula complete__exists_formula();
        protected:
            /* Should next next call to \member{report__formula} be skipped?*/
            bool skip_next____report__formula;

            /* How deep are we while parsing a formula? -- Count of
             * opening brackets.*/
            int formula_parsing_level;

            /* Keeping track of subformulae while parsing a formula.*/
            std::map<int, Planning::Formula::Subformulae> subformulae;
            
//             std::map<Predicate_Name, std::map<Variable, Types> > predicatex_variablex_types;
//             std::map<Action_Name, std::map<Variable, Types> > actionx_variablex_types;

            std::stack<Planning::enum_types> formula_type;

            /* Untyped variables in the fragment of the formula
             * currently being parsed. */
            Planning::Variables variables;

            /* Computer generated predicates whose truth value is
             * derived from that of other predicates. */
            Planning::Derived_Predicates derived_Predicates__artificial;
            
            /* Parsed action name.*/
            Planning::Action_Name action_Name;

            /* Parsed predicate name.*/
            Planning::Predicate_Name predicate_Name;
            
            /* Parsed function name.*/
            Planning::State_Function_Name state_Function_Name;
            
            /* Parsed perceptual function name.*/
            Planning::Perceptual_Function_Name perceptual_Function_Name;
            
            /* Parsed percept name.*/
            Planning::Percept_Name percept_Name;
            
            /*Was the last number parsed a \type{double}?*/
            bool last_number_parsed_was_double;

            /* If \member{last_number_parsed_was_double} is true, then
             * the following member (\member{last_number__double})
             * should be considered the value of the last number
             * parsed.*/
            double last_number__double;
            
            /* If \member{last_number_parsed_was_double} is false,
             * then the following member (\member{last_number__int})
             * should be considered the value of the last number
             * parsed.*/
            int last_number__int;
            
            /* Arguments as have been parsed thus far.*/
            Planning::Typed_Arguments typed_Arguments;
            
            /* Symbols such as predicates, percepts, actions,
             * quantified preconditions, etc take a list of typed
             * variable arguments.*/
            Planning::Argument_List argument_List;            
        };
    }
}


#endif
