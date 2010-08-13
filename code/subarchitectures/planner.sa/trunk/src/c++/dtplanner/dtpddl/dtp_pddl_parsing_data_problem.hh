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
#ifndef DTP_PDDL_PARSING_DATA_PROBLEM_HH
#define DTP_PDDL_PARSING_DATA_PROBLEM_HH


#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_constants.hh"
#include "dtp_pddl_parsing_data_formula.hh"

 
namespace Planning
{
    namespace Parsing
    {
        class Problem_Data : public Constants_Data, /*think PDDL objects*/
                             public Formula_Data /*think PDDL preconditions, derived-predicates, postconditions, etc.*/
        {
        public:
            enum Objective {maximise, minimise};
            
            friend std::ostream& std::operator<<(std::ostream&, const Planning::Parsing::Problem_Data&);
            Problem_Data(CXX__PTR_ANNOTATION(Domain_Data)& domain_Data);

            const CXX__PTR_ANNOTATION(Domain_Data)& get__domain_Data() const;
            CXX__PTR_ANNOTATION(Domain_Data) get__domain_Data();
            
            void report__minimisation_objective();
            void report__maximisation_objective();
            
            void report__starting_state();
            void report__objective_function();
            void report__goal_formula();
            
            /* Associate a domain model with name
             * \argument{Domain_Name} with this problem.*/
            void reset__domain_Data(const Planning::Domain_Name&);
            /*(see \member{reset__domain_Data(const Planning::Domain_Name&)})*/
            void reset__domain_Data(const std::string&);
            /* Associate a domain model with this problem.*/
            void reset__domain_Data(CXX__PTR_ANNOTATION(Domain_Data)& in__domain_Data);
            
            void add__problem_Name(const std::string&);
            const Planning::Problem_Name& get__problem_Name() const;

            /*For Freiburg Jun 2010 */
            void report__observations(const std::vector<std::string>& observationSeq);
            Planning::Formula::Action_Proposition get__prescribed_action();



            Planning::Formula::Subformula get__starting_state() const;
            
            const Planning::Formula::Subformula& get__objective_function() const;


            /******************************************************************************************
             ******************************************************************************************
             ******************************************************************************************
             ************************************ STATIC ANALYSIS *************************************
             ******************************************************************************************
             ******************************************************************************************
             ******************************************************************************************/
            
            bool has_static_value(Formula::State_Ground_Function&) const;
            bool has_static_value(Formula::Subformula
                                  ,  const Planning::Assignment& assignment) const;
            
            static const Planning::Assignment EMPTY_ASSIGNMENT;
            
            template<typename T>
            T read__static_value(Formula::Subformula modification,
                                 const Planning::Assignment& assignment = EMPTY_ASSIGNMENT) const;
            
            template<typename T>
            T read__static_value(const Planning::Formula::State_Ground_Function&) const;


            typedef ID_TYPE Predicate_Index;

            Planning::Formula::Subformula X_constant;


            typedef std::tr1::unordered_set<Planning::Argument_List
                                            , boost::hash<Planning::Argument_List> >
            Cached_Partial_Assignment_Unsatisfiability;

            
            typedef std::tr1::unordered_set<Planning::Argument_List
                                            , boost::hash<Planning::Argument_List> >
            Cached_Partial_Assignment_Satisfiability;
        
            typedef std::map<Predicate_Index
                             , std::tr1::tuple<Cached_Partial_Assignment_Satisfiability
                                               , Cached_Partial_Assignment_Unsatisfiability> > Cached_Predicate_Satisfiability;

            
            
            /* Translates constant symbols from the base to be associated
             * with the planning problem. First argument is the base of
             * the translation, second argument is the result of the
             * translation. Result indicates if a translation occurred. */
            bool translate_to_problem_arguments(const Planning::Argument_List&, Planning::Argument_List&) const;
            bool translate_to_problem_arguments(const Planning::Constant_Arguments&, Planning::Constant_Arguments&) const;
            
            /* Groundability tests for problem and domain symbols.*/
            bool statically_true__starting_always_true(CXX__deref__shared_ptr<Planning::Formula::State_Proposition>&) const;
            bool statically_true__starting_always_true(CXX__deref__shared_ptr<Planning::Formula::State_Predicate>&) const;
            
            /* Saves time for future invocations by caching results in
             * \member{cached__statically_false__starting_always_false}.
             *
             * ASSUMPTION: Problem starting state is _not_ changed
             * during (or after) a call to this member.*/
            bool statically_false__starting_always_false(CXX__deref__shared_ptr<Planning::Formula::State_Predicate>&) const;
            
            bool statically_false__starting_always_false(CXX__deref__shared_ptr<Planning::Formula::State_Proposition>&) const;

        private:
            /* Cached results for \member{statically_false__starting_always_false}.*/
            mutable Cached_Predicate_Satisfiability cached__statically_false__starting_always_false;


            
        private:
            /*Description of the planning goal.*/
            Planning::Formula::Subformula goal_formula;
            
            /*Last starting state formula parsed.*/
            Planning::Formula::Subformula starting_state;
            
            /*The function/fluent that \member{objective} applies to.*/
            Planning::Formula::Subformula objective_function;
            
            /*Planning objective*/
            Objective objective;
            
            /* Each problem has a name. The
             * \member{domain_Data::domain_name} and
             * \member{problem_name} uniquely identify a problem.*/
            Planning::Problem_Name problem_Name;

            /*Problem is an instance of a domain.*/
            CXX__PTR_ANNOTATION(Domain_Data) domain_Data;
        public:
            typedef Problem_Identifier Identifier;
        };
    }
}


namespace std
{
    std::ostream& operator<<(std::ostream&
                             , const Planning::Parsing::Problem_Data::Objective&);
}


#endif
