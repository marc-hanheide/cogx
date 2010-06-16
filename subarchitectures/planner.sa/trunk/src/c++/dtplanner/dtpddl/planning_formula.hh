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
#ifndef PLANNING_FORMULA_HH
#define PLANNING_FORMULA_HH

#include "planning_symbols.hh"

namespace Planning
{
    
    namespace Formula
    {
        typedef basic_types__vector Subformulae;
        typedef CXX__deref__shared_ptr<basic_type> Subformula;
        
        namespace Printing
        {
            
            std::ostream& operator<<(std::ostream&, const std::tr1::tuple<Subformulae>&);
            std::ostream& operator<<(std::ostream&, const std::tr1::tuple<Subformula >&);
            std::ostream& operator<<(std::ostream&, const Constant_Arguments&);
            std::ostream& operator<<(std::ostream&, const Argument_List&);
        }

        /* Is an empty formula is treated equivalent to TRUE (i.e.,
         * TOP).*/
        class Vacuous : public type_wrapper<enum_types::vacuous
                                            , void*>
        {PRINTING;};

        template<int ID_VAL, typename NAMING_TYPE>
        class Predicate : public type_wrapper< ID_VAL
                                               , NAMING_TYPE
                                               , Argument_List>
        {
        public: typedef type_wrapper<ID_VAL, NAMING_TYPE, Argument_List> Parent;
            PRINTING;
        };
        
        template<int ID_VAL, typename NAMING_TYPE>
        class Proposition : public type_wrapper< ID_VAL
                                                 , NAMING_TYPE
                                                 , Constant_Arguments>
        {
        public: typedef type_wrapper<ID_VAL, NAMING_TYPE, Constant_Arguments> Parent;
            PRINTING;
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
        MAKE_CONSTANT_ATOM(Observational_Proposition, enum_types::observational_proposition, Planning::Percept_Name);
        MAKE_VARIABLE_ATOM(Observational_Predicate, enum_types::observational_predicate, Planning::Percept_Name);
        
        MAKE_CONSTANT_ATOM(State_Ground_Function, enum_types::state_ground_function, Planning::Function_Name);
        MAKE_VARIABLE_ATOM(State_Function, enum_types::state_function, Planning::Function_Name);
        MAKE_CONSTANT_ATOM(Perceptual_Ground_Function, enum_types::perceptual_ground_function, Planning::Perceptual_Function_Name);
        MAKE_VARIABLE_ATOM(Perceptual_Function, enum_types::perceptual_function, Planning::Perceptual_Function_Name);

        class Conjunction : public type_wrapper<enum_types::conjunction
                                                , Subformulae>
        {PRINTING;
        };
        
        class Disjunction : public type_wrapper<enum_types::disjunction
                                                , Subformulae>
        {PRINTING;
        };
        
        class Negation : public type_wrapper<enum_types::negation
                                                , Subformula >
        {PRINTING;
        };
        
        class Forall : public type_wrapper<enum_types::forall
                                           , Variable
                                           , Type
                                           , Subformula >
        {
        };

        class Exists : public type_wrapper<enum_types::exists
                                           , Variable
                                           , Type
                                           , Subformula >
        {
        };
        

        
        class Number : public type_wrapper<enum_types::number, double>
        {
        public:
            double get__value() const;
        };
        
        
        class Increase : public type_wrapper<enum_types::increase, Subformula, Subformula>
        {PRINTING;
        };
        
        typedef std::vector<Number> numbers__vector;
        
        class Probabilistic : public type_wrapper<enum_types::probabilistic_effect
                                                  , Subformulae
                                                  , numbers__vector>
        {PRINTING;

        public:
            numbers__vector get__probabilities() const ;
            Subformulae get__formulae() const ;
            
            /* Does this probabilistic effect make sense -- i.e. do the
             * elements in \member{numbers__vector} sum to 1.*/
            bool sanity() const;
        };
        
    }
}

#include "planning_formula__TEMPLATES.hh"

#endif
