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
#ifndef PLANNING_FORMULA_HH
#define PLANNING_FORMULA_HH

#include "planning_symbols.hh"

namespace Planning
{
    
    namespace Formula
    {
        typedef CXX__deref__shared_ptr__visitable<basic_type> Subformula;
        typedef std::vector<Subformula> Subformulae;
//         typedef basic_types__vector Subformulae;
//         typedef CXX__deref__shared_ptr<basic_type> Subformula;
        
        namespace Printing
        {
            
            std::ostream& operator<<(std::ostream&, const std::tr1::tuple<Subformulae>&);
            std::ostream& operator<<(std::ostream&, const std::tr1::tuple<Subformula >&);
            std::ostream& operator<<(std::ostream&, const Constant_Arguments&);
            std::ostream& operator<<(std::ostream&, const Argument_List&);
        }

        /* Is an empty formula is, during processing, treated
         * equivalent to TRUE (i.e., TOP).*/
        class Vacuous : public type_wrapper<enum_types::vacuous
                                            , void*>
        {PRINTING;};

        class True : public type_wrapper<enum_types::formula_true
                                         , void*>
        {PRINTING;};
        
        class False : public type_wrapper<enum_types::formula_false
                                            , void*>
        {PRINTING;};

        template<int ID_VAL, typename NAMING_TYPE>
        class Predicate : public type_wrapper< ID_VAL
                                               , NAMING_TYPE
                                               , Argument_List>
        {
        public: typedef type_wrapper<ID_VAL, NAMING_TYPE, Argument_List> Parent;
            PRINTING;
        public:
            const NAMING_TYPE& get__name() const {return std::tr1::get<0>(Parent::contents());};
            const Argument_List& get__arguments() const {return std::tr1::get<1>(Parent::contents());};

            uint get__arity() const{return std::tr1::get<1>(Parent::contents()).size();};
        };
        
        template<int ID_VAL, typename NAMING_TYPE>
        class Proposition : public type_wrapper< ID_VAL
                                                 , NAMING_TYPE
                                                 , Constant_Arguments>
        {
        public: typedef type_wrapper<ID_VAL, NAMING_TYPE, Constant_Arguments> Parent;
            PRINTING;
        public:
            const NAMING_TYPE& get__name() const {return std::tr1::get<0>(Parent::contents());};
            const Constant_Arguments& get__arguments() const {return std::tr1::get<1>(Parent::contents());};
            uint get__arity() const{return std::tr1::get<1>(Parent::contents()).size();};
        };

#define MAKE_CONSTANT_ATOM(TYPE_NAME, ID_VAL, NAMING_TYPE)              \
        class TYPE_NAME : public Proposition<ID_VAL, NAMING_TYPE>{}    \
                
#define MAKE_VARIABLE_ATOM(TYPE_NAME, ID_VAL, NAMING_TYPE)              \
        class TYPE_NAME : public Predicate<ID_VAL, NAMING_TYPE>{}      \
        

//         MAKE_CONSTANT_ATOM(_Proposition, enum_types::_proposition, _Name);
//         MAKE_VARIABLE_ATOM(_Predicate, enum_types::_predicate, _Name);

        MAKE_CONSTANT_ATOM(Action_Proposition, enum_types::action_proposition, Planning::Action_Name);
        MAKE_VARIABLE_ATOM(Action_Predicate, enum_types::action_predicate, Planning::Action_Name);
        MAKE_CONSTANT_ATOM(State_Proposition, enum_types::state_proposition, Planning::Predicate_Name);
        MAKE_VARIABLE_ATOM(State_Predicate, enum_types::state_predicate, Planning::Predicate_Name);

        
        MAKE_CONSTANT_ATOM(Perceptual_Proposition, enum_types::perceptual_proposition, Planning::Percept_Name);
        MAKE_VARIABLE_ATOM(Perceptual_Predicate, enum_types::perceptual_predicate, Planning::Percept_Name);
        
        MAKE_CONSTANT_ATOM(Observational_Proposition, enum_types::observational_proposition, Planning::Observation_Name);
        MAKE_VARIABLE_ATOM(Observational_Predicate, enum_types::observational_predicate, Planning::Observation_Name);
        
        MAKE_CONSTANT_ATOM(State_Ground_Function, enum_types::state_ground_function, Planning::State_Function_Name);
        MAKE_VARIABLE_ATOM(State_Function, enum_types::state_function, Planning::State_Function_Name);
        MAKE_CONSTANT_ATOM(Perceptual_Ground_Function, enum_types::perceptual_ground_function, Planning::Perceptual_Function_Name);
        MAKE_VARIABLE_ATOM(Perceptual_Function, enum_types::perceptual_function, Planning::Perceptual_Function_Name);

        typedef std::set<Action_Proposition> Action_Propositions;
        typedef std::set<Action_Predicate> Action_Predicates;
        typedef std::set<State_Proposition> State_Propositions;
        typedef std::set<State_Predicate> State_Predicates;
//         typedef std::set<Observational_Proposition> Observational_Propositions;
        typedef std::set<Observational_Predicate> Observational_Predicates;
        typedef std::set<State_Ground_Function> State_Ground_Functions;
        typedef std::set<State_Function> State_Functions;
        typedef std::set<Perceptual_Ground_Function> Perceptual_Ground_Functions;
        typedef std::set<Perceptual_Function> Perceptual_Functions;

        typedef CXX__deref__shared_ptr<Observational_Proposition> Observational_Proposition__Pointer;
        typedef std::vector<Observational_Proposition__Pointer> List__Observational_Propositions;
        typedef std::set<Observational_Proposition__Pointer> Observational_Propositions;
        
        typedef CXX__deref__shared_ptr<Perceptual_Proposition> Perceptual_Proposition__Pointer;
        typedef std::vector<Perceptual_Proposition__Pointer> List__Perceptual_Propositions;
        typedef std::set<Perceptual_Proposition__Pointer> Perceptual_Propositions;


        
        typedef CXX__deref__shared_ptr<State_Proposition> State_Proposition__Pointer;
        typedef std::vector<State_Proposition__Pointer> List__State_Propositions;

        
        
        class Conjunction : public type_wrapper<enum_types::conjunction
                                                , Subformulae>
        {PRINTING;
        public:
            const Subformulae& get__subformulae() const;
            Subformulae get__subformulae();
        };
        
        class Disjunction : public type_wrapper<enum_types::disjunction
                                                , Subformulae>
        {PRINTING;
        public:
            const Subformulae& get__subformulae() const;
            Subformulae get__subformulae();
        };
        
        class Negation : public type_wrapper<enum_types::negation
                                                , Subformula >
        {PRINTING;
        public:
            const Subformula& get__subformula() const;
            Subformula get__subformula();
        };
        
        class Forall : public type_wrapper<enum_types::forall
                                           , Variable
                                           , Type
                                           , Subformula >
        {
        public:
            const Variable& get__variable() const;
            Variable get__variable();

            const Type& get__variable_type() const;
            Type get__variable_type();
            
            const Subformula& get__subformula() const;
            Subformula get__subformula();
        };

        class Exists : public type_wrapper<enum_types::exists
                                           , Variable
                                           , Type
                                           , Subformula >
        {
        public:
            const Variable& get__variable() const;
            Variable get__variable();

            const Type& get__variable_type() const;
            Type get__variable_type();
            
            const Subformula&  get__subformula() const;
            Subformula get__subformula();
        };
        

        
        class Number : public type_wrapper<enum_types::number, double>
        {
        public:
            double get__value() const;
        };

        template<int ID_VAL>
        class Function_Modifier : public type_wrapper<ID_VAL, Subformula, Subformula>
        {
        public:
            
            /* Function that is being acted on by operator with ID_VAL
             * (see \module{planning_types_enum.hh})*/
            Subformula get__subject() const {return std::tr1::get<0>(this->contents());}

            /* Formulae that can be evaluated at a planning state to
             * give the number by which the \tupelem{subject} is
             * modified.*/
            Subformula get__modification() const {return std::tr1::get<1>(this->contents());}
            
        private:
            std::ostream& operator<<(ostream&o) const
            {
                o<<"("<<get__operator_type_as_string()
                 <<" "<<std::tr1::get<0>(this->contents())<<" "<<std::tr1::get<1>(this->contents());
                return o<<")"<<std::endl;
            }
        protected:
            virtual std::string get__operator_type_as_string() const = 0;
        };
        
        class Increase : public Function_Modifier<enum_types::increase>
        {
        protected:
            std::string get__operator_type_as_string() const;
        };
        
        class Decrease : public Function_Modifier<enum_types::decrease>
        {
        protected:
            std::string get__operator_type_as_string() const;
        };
        
        class Assign : public Function_Modifier<enum_types::assign> 
        {
        protected:
            std::string get__operator_type_as_string() const;
        };


        class Equality_Test : public Function_Modifier<enum_types::equality_test> 
        {
        protected:
            std::string get__operator_type_as_string() const;
        };

        
        class Conditional_Effect : public type_wrapper<enum_types::conditional_effect, Subformula, Subformula>
        {PRINTING;
        public:
            Subformula get__condition() const;
            Subformula get__effect() const;
        };


//         /* Numbers can appear in list elements in a domain or task
//          * description (see \class{Probabilistic})*/
//         typedef std::vector<Number> numbers__vector;
        
        class Probabilistic : public type_wrapper<enum_types::probabilistic_effect
                                                  , Subformulae
                                                  , Subformulae> //numbers__vector>
        {PRINTING;

        public:
            Subformulae get__probabilities() const ;
            Subformulae get__formulae() const ;
            
            /* MEANINGLESS (always TRUE) IF THE FORMULA from
             * \member{get__probabilities()} ARE NOT NUMERIC. Does this
             * probabilistic effect make sense -- i.e. do the elements
             * in \member{numbers__vector} sum to 1.*/
            bool sanity() const;
            
            /* MEANINGLESS (always TRUE) IF THE FORMULA from
             * \member{get__probabilities()} ARE NOT NUMERIC. Does the
             * sum of elements in \tupelem{probabilities} sum to a
             * value less-than-or-equal-to 1. If so, we suppose it is
             * a valid PDDL formula.*/
            bool leq1() const;
        };
        
    }
}

#include "planning_formula__TEMPLATES.hh"

#endif
