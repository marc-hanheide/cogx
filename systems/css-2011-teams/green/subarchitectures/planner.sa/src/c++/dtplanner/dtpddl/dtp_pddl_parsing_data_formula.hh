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
            /* 
             * formula_parsing_level ---0--- Depth in the parse tree
             *
             * skip_next____report__formula ---false--- should we
             * ignore a subsequent call to \member{report__formula}?
             *
             * last_number_parsed_was_double ---false--- Was the last
             * numeric symbol parsed a double representated real
             * number?
             *
             * parsing_initial_state ---false--- Are we parsing a
             * problem's initial state?
             *
             * in_delete_context ---false--- Are we parsing a delete
             * effect of a PDDL operator?
             *
             * in_modification_context ---false--- Are we parsing a
             * 'assign', 'decrement', or 'increment' modification
             * instruction to a numeric valued function symbol in an
             * operator effect?
             *
             * in_effect_context ---false--- Are we parsing a decriton
             * of an operator effect?
             */
            Formula_Data();
            
            template<typename PROPOSITION_SYMBOL, typename Symbol_Name >
            void using__symbol_name(const Symbol_Name& symbol_name, ID_TYPE index);
            
            /* Formulae that are subsequently parsed have as their
             * subject the initial state.  Some formula describe an
             * action's pre- and post-conditions, others describe
             * derived predicates. Depending on the context in which a
             * formula is being parsed the PDDL-tokens have a
             * different interpretation. For example, in PDDL the '='
             * token in a descirption of an initial-state has the
             * semantics of the 'assign' operator in an action
             * effect. On the other hand, in a 'precondition' the '='
             * token has the semantics of an equality test.  */
            void report__enter_parsing_initial_state();
            
            /* (see \member{report__enter_parsing_initial_state})
             * Formulae that are subsequently parsed DO NOT have as
             * their subject the initial state (see
             * \member{report__enter_parsing_initial_state}).*/
            void report__exit_parsing_initial_state();


            /* Report that we are parsing a formula that describes a
             * PDDL operator/action effect.*/
            void report__enter_parsing_effect_context();
            
            /* (see \member{report__enter_parsing_effect_context})
             * Report that we are not parsing a formula that describes
             * an operator effect.*/
            void report__exit_parsing_effect_context();

            /* Pushing \member{typed_Arguments} (a description of the
             * variable and constant arguments to a first-order
             * symbol) onto
             * \member{Types_Data::stack_of__Typed_Arguments}. This
             * consumes, and thus re-initialises
             * \member{typed_Arguments}*/
            void stack__typed_Arguments();

            /* */
            void report__perceptual_function_name(const std::string& str);
            void report__percept_name(const std::string& str);
            void report__state_function_name(const std::string& str);
            void report__predicate_name(const std::string& str);
            
            
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
                    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                        (PREDICATE_SYMBOL
                         , atomic_symbol
                         , symbol_name
                         , argument_List);
                    this->using__symbol_name
                        <PREDICATE_SYMBOL
                        , Symbol_Name>
                        (symbol_name,
                         atomic_symbol->get__id());
                    
                    subformulae[formula_parsing_level].push_back(atomic_symbol);
                } else {
                    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                        (PROPOSITION_SYMBOL
                         , atomic_symbol
                         , symbol_name
                         , constant_Arguments);
                    subformulae[formula_parsing_level].push_back(atomic_symbol);

                    this->using__symbol_name
                        <PROPOSITION_SYMBOL
                        , Symbol_Name>
                        (symbol_name,
                         atomic_symbol->get__id());
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

            /* Do predicates, or propositions with name
             * \argument{Predicate_Name} occure in an actions add
             * effect.*/
            bool in_add_effect(const Predicate_Name&) const;

            /* Do predicates, or propositions, with name
             * \argument{Predicate_Name} occure in an actions delete
             * effect.*/
            bool in_delete_effect(const Predicate_Name&) const;

            /* Does an occurrence of symbols
             * \argument{State_Function_Name} occur in the context of
             * a modifier? (i.e, assign, decrement, increment, etc.)*/
            bool is_static_fluent(const Planning::State_Function_Name&) const;
            
            /* IF this is a base of \class{Problem_Data}, THEN
             * \member{state_propositions__parsed} contains all the
             * propositions that _can_ be true in the problem starting
             * state. As usual we make the closed-world assumption,
             * and consequently, anything that is not indexed by
             * \member{state_propositions__parsed} is necessarily
             * false in any starting state. Because some propositions
             * can only become false, and others can only become true,
             * we can sometimes know if a predicate is satisfiable in
             * any planning state, just by examining what is available
             * in the starting states.
             *
             * Here \member{statically_satisfiable} assumes that its
             * \argument{State_Predicate} cannot be made FALSE by any
             * planning actions. Contrariwise,
             * \member{statically_unsatisfiable} supposes that no
             * action can make its \argument{State_Predicate}
             * TRUE. Therefore, these methods are able to determine
             * the possible satisfiability/unsatisfiability
             * (necessary, but not sufficient analysis) of some
             * propositions.
             *
             * if \member{statically_satisfiable} returns TRUE, then
             * the \argument{State_Predicate} could indeed be ground
             * to something that is TRUE in a starting state. If it
             * returns FALSE, then no such possibility is available --
             * i.e., necessarily FALSE.
             *
             * Therefore, \member{statically_satisfiable} gives the
             * opposite answer to \member{statically_satisfiable}.
             */
            bool statically_satisfiable(const Planning::Formula::State_Predicate&) const;
            bool statically_unsatisfiable(const Planning::Formula::State_Predicate&) const;

            /* Can a ground instance of \argument{State_Predicate} be
             * equal to \argument{State_Proposition}? Here we assume
             * that the \argument{State_Predicate} symbol is static,
             * and therefore can examine the problem starting states
             * to see what is possible.
             *
             * ASSUMES: Planning symbols have the same name and arity.
             */
            bool potential_match_via_an_assignment(const Planning::Formula::State_Predicate&,
                                                     const Planning::Predicate_Name& State_Proposition__name,
                                                     const Constant_Arguments& State_Proposition__arguments) const;

            virtual bool is_type__double(const Planning::State_Function_Name&) const;
            virtual bool is_type__int(const Planning::State_Function_Name&) const;
            virtual bool is_type__double(const Planning::Perceptual_Function_Name&) const;
            virtual bool is_type__int(const Planning::Perceptual_Function_Name&) const;
            virtual bool is_type__number(const Planning::State_Function_Name&) const;
            virtual bool is_type__number(const Planning::Perceptual_Function_Name&) const;
        protected:
            
            /*Cached value of starting state functional assignments -- ONLY USED BY PARENTS.*/
            std::map<Formula::State_Ground_Function, double> static_ground_double_function;
            std::map<Formula::State_Ground_Function, int> static_ground_int_function;

        private:
            
            /* Does \member{subformulae} have elements at \argument{index}.*/
            bool check__exists_parsed_subformulae(int index) const;
            
            /* Are there \argument{count} elements at
             * \member{subformulae} element at \argument{index}.*/
            bool check__cardinality_constraint_on_subformulae_at_index
            (int count, int index) const;
            
            /*For \argument{quantifier} see \enum{enum_types} from \module{planning_symbols}.*/
            Formula::Subformula complete__quantified_formula(int quantifier);
            /*(see \member{complete__quantified_formula(forall)})*/
            Formula::Subformula complete__forall_formula();
            /*(see \member{complete__quantified_formula(exists)})*/
            Formula::Subformula complete__exists_formula();

        public:
            const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >& get__state_functions__parsed() const;
            
            const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >& get__perceptual_functions__parsed() const;
            
            const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >& get__state_ground_functions__parsed() const;
            
            const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >& get__perceptual_ground_functions__parsed() const;
            
            const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& get__state_propositions__parsed() const;
            
            const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& get__state_predicates__parsed() const;
            
            const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& get__perceptual_propositions__parsed() const;
            
            const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& get__perceptual_predicates__parsed() const;
            
            const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& get__deleted__state_propositions__parsed() const;
            
            const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& get__deleted__state_predicates__parsed() const;
            
            const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& get__deleted__perceptual_propositions__parsed() const;
            
            const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& get__deleted__perceptual_predicates__parsed() const;
            
            const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& get__added__state_propositions__parsed() const;
            
            const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& get__added__state_predicates__parsed() const;
            
            const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& get__added__perceptual_propositions__parsed() const;
            
            const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& get__added__perceptual_predicates__parsed() const;
            
            
            
            const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >&
            get__modified_in_effect__state_functions__parsed() const;
            
            const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >&
            get__modified_in_effect__state_ground_functions__parsed() const;
            
            const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >&
            get__modified_in_effect__perceptual_functions__parsed() const;
            
            const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >&
            get__modified_in_effect__perceptual_ground_functions__parsed() const;
        protected:
            /* What propositions are necessarily true in the starting states?*/
            std::set<ID_TYPE>  starting_state_propositions;
            
            /* Store all the atomic symbols that occur in the
             * formulae parsed by this.*/
            std::map<Planning::State_Function_Name, std::set<ID_TYPE> > state_functions__parsed;
            std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> > perceptual_functions__parsed;
            std::map<Planning::State_Function_Name, std::set<ID_TYPE> > state_ground_functions__parsed;
            std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> > perceptual_ground_functions__parsed;
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > state_propositions__parsed;
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > state_predicates__parsed;
            std::map<Planning::Percept_Name, std::set<ID_TYPE> > perceptual_propositions__parsed;
            std::map<Planning::Percept_Name, std::set<ID_TYPE> > perceptual_predicates__parsed;


            /* Boolean symbols are "added" and Boolean symbols that
             * are "deleted" in formulae. This store makes later
             * detection of static Boolean symbols possible.*/
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > deleted__state_propositions__parsed;
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > deleted__state_predicates__parsed;
            std::map<Planning::Percept_Name, std::set<ID_TYPE> > deleted__perceptual_propositions__parsed;
            std::map<Planning::Percept_Name, std::set<ID_TYPE> > deleted__perceptual_predicates__parsed;
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > added__state_propositions__parsed;
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > added__state_predicates__parsed;
            std::map<Planning::Percept_Name, std::set<ID_TYPE> > added__perceptual_propositions__parsed;
            std::map<Planning::Percept_Name, std::set<ID_TYPE> > added__perceptual_predicates__parsed;

            

            /* While parsing we also keep information about what
             * functional symbols are "modified" in formulae. This
             * makes later detection of static function (PDDL, so
             * called fluent=function) symbols possible.*/
            std::map<Planning::State_Function_Name, std::set<ID_TYPE> > modified_in_effect__state_functions__parsed;
            std::map<Planning::State_Function_Name, std::set<ID_TYPE> > modified_in_effect__state_ground_functions__parsed;
            std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> > modified_in_effect__perceptual_functions__parsed;
            std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> > modified_in_effect__perceptual_ground_functions__parsed;

            /* How deep are we while parsing a formula?*/
            int formula_parsing_level;

            /* Should next next call to \member{report__formula} be skipped?*/
            bool skip_next____report__formula;

            /*Was the last number parsed a \type{double}?*/
            bool last_number_parsed_was_double;

            /* Does the formula being based have as its subject the
             * initial state of a planning problem?*/
            bool parsing_initial_state;
            
            /* This Boolean is true when a propositional symbol being
             * parsed is occurring in a delete context. That is, if
             * the formula being parse is an effect formula, and the
             * proposition that was parsed was parsed in a delete
             * context (will be deleted when the corresponding action
             * is executed).
             *
             * (see \member{report__formula} and \member{report__not_formula})
             *
             * INITIALLY "FALSE".*/
            bool in_delete_context;
            
            /* Are with below the context of a function modifier --
             * i.e., assign, increase, decrease.
             *
             * INITIALLY "FALSE".*/
            bool in_modification_context;
            
            /* Are we parsing a formula that describes the effects of
             * an operator?*/
            bool in_effect_context;
            
            /* Keeping track of subformulae while parsing a formula.*/
            std::map<int, Planning::Formula::Subformulae> subformulae;
            
            /* When a report of the type of formula being parsed is
             * received ---for example, by via a call to
             * \member{report__probabilistic_formula}---, that
             * reported type is stored here. */
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

/* They proved that if you quit smoking, it will prolong your
 * life. What they haven't proved is that a prolonged life is a good
 * thing. I haven't seen the stats on that yet. ... I'm available for
 * children's parties, by the way.
 *
 * -- Bill Hicks, Ribaldrous Comedian from the USA,  1961-94.
 */
