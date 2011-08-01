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

            Domain_Data();

//             void stack__typed_Arguments();


            void add__observation_execution_precondition();
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

            
            void add__requirement(const std::string& str);
            
            void add__derived_predicate_header();
            void commit__derived_predicate();
        public:

            /*Obtains the type of the argument variable from the indicated structure.*/
            Planning::Types find__derived_Predicate_Header(const Planning::Variable&) const;
            Planning::Types find__action_Header(const Planning::Variable&) const;
            Planning::Types find__type_of_variable(const Planning::Variable&) const;

            void add__percept();
            void add__predicate();

            /* A function takes arguments, those are objects over the
             * domain of the function. For example, (total-cost) has
             * an empty range. Another example, (cost ?c - color) -
             * number is a function with domain color.*/
            void report__state_function_domain();
            /* Perceptual function counterpart to
             * \member{report__state_function_domain}*/
            void report__perceptual_function_domain();
            void add__state_function();
            void add__perceptual_function();
 
            const Planning::Action_Schemas& get__action_Schemas() const;
            Planning::Action_Schemas& get__action_Schemas();
            
            const Planning::Derived_Predicates& get__derived_Predicates() const;
            Planning::Derived_Predicates& get__derived_Predicates();

            
            const Planning::Derived_Percepts& get__derived_Percepts() const;
            Planning::Derived_Percepts& get__derived_Percepts();

            const Planning::Observation_Schemas& get__observation_Schemas() const;
            Planning::Observation_Schemas& get__observation_Schemas();
            

            
            bool is_type__double(const Planning::State_Function_Name&) const;
            bool is_type__int(const Planning::State_Function_Name&) const;
            bool is_type__double(const Planning::Perceptual_Function_Name&) const;
            bool is_type__int(const Planning::Perceptual_Function_Name&) const;
            bool is_type__number(const Planning::State_Function_Name&) const;
            bool is_type__number(const Planning::Perceptual_Function_Name&) const;
        private:
            Planning::Type double__constant;
            Planning::Type int__constant;
            Planning::Type number__constant;
            std::map<Planning::State_Function_Name
                     , Planning::Types> range_of_state_function;
            std::map<Planning::Perceptual_Function_Name
                     , Planning::Types> range_of_perceptual_function;
            
            /*(see \member{report__state_function_domain})*/
            Planning::Typed_Arguments state_function_domain_specification;

            /*(see \member{report__perceptual_function_domain})*/
            Planning::Typed_Arguments perceptual_function_domain_specification;
            
            /*Domain observations.*/
            Planning::Observation_Schemas observation_Schemas;
            
//             /*Last observation action precondition.*/
//             Planning::Formula::Subformula observation_execution;
            Planning::Formula::Subformula observation_execution_precondition;
            
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
            
            /* Is the entry in \member{action_precondition} relevant
             * to the current parsing context? (initially false, and
             * reset to false in \member{add__action()})*/
            bool got__action_precondition;
            
            /* Is the entry in \member{action_effect} relevant to the
             * current parsing context? (initially false and reset to
             * false in \member{add__action()})*/
            bool got__action_effect;

            /* Is the entry in \member{observation_effect} relevant to
             * the current parsing context? (initially false, and
             * reset to false in \member{add__observation()})*/
            bool got__observation_precondition;
            
            /* Is the entry in \member{observation_effect} relevant to
             * the current parsing context? (initially false, and
             * reset to false in \member{add__observation()})*/
            bool got__observation_execution_precondition;
            
            /* Is the entry in \member{observation_effect} relevant to
             * the current parsing context? (initially false, and
             * reset to false in \member{add__observation()})*/
            bool got__observation_effect;
            

            
            /*Parsing the headers of derived predicates.*/
            Planning::Derived_Predicate_Header derived_Predicate_Header;
            
            /* Predicates whose truth value is derived from that of
             * other predicates. */
            Planning::Derived_Predicates derived_Predicates;

            /* For all predicates that are derived, we map their names
             * to the indexes where their specifications occur.*/
            std::map<Planning::Predicate_Name, std::set<ID_TYPE> > derived__state_predicates__parsed;
            
            /*Parsing the headers of derived predicates.*/
            Planning::Derived_Percept_Header derived_Percept_Header;
            
            /* Predicates whose truth value is derived from that of
             * other predicates. */
            Planning::Derived_Percepts derived_Percepts;


            
            
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
            
            /* Set of state functions parsed.*/
            Planning::State_Function_Descriptions state_Function_Descriptions;
            
            /* Set of perceptual functions parsed.*/
            Planning::Perceptual_Function_Descriptions perceptual_Function_Descriptions;
            
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

/* And again, this is a little to rich for my taste. ... Somehow if
 * you have to foo that often somebody is not fooing right.
 *
 *  -- Rob Pike ---advertising Google Go and talking about C++ boost
 * memory management templates---, keynote speech titled "Public
 * Static Void" at OSCON (O'Reilly's Open Source CONvention), 2010.
 *
 *
 *
 *
 *
 *
 * ... == I think things have gotten a bit.. um..  too sophisticated
 * for.. anyone but the most expert programmers. And.. okay those are
 * both C++, maby its too easy a target -- It's a complicated and very
 * large language. But it's not the only language out there that
 * causes issues for people.
 */
