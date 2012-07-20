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
#ifndef PLANNING_DERIVED_PREDICATE_HH
#define PLANNING_DERIVED_PREDICATE_HH


#include "planning_formula.hh"

namespace Planning
{
    template<int ID_VAL, typename NAMING_TYPE>
    class First_Order_Derived_Symbol_Header : public type_wrapper<ID_VAL
                                                         , NAMING_TYPE
                                                         , Typed_Arguments
                                                         >
    {
    public: typedef type_wrapper<ID_VAL, NAMING_TYPE, Typed_Arguments> Parent;
        PRINTING;
    public:
        NAMING_TYPE get__name() const;
        Planning::Typed_Arguments get__arguments() const ;
    };
    
    class Derived_Predicate_Header
        : public First_Order_Derived_Symbol_Header
        <enum_types::derived_predicate_header
         , Planning::Predicate_Name>{};
    
    class Derived_Percept_Header
        : public First_Order_Derived_Symbol_Header
        <enum_types::derived_percept_header
         , Planning::Percept_Name>{};
    
    
    template<int ID_VAL, typename NAMING_TYPE>
    class First_Order_Derived_Symbol
        : public type_wrapper<ID_VAL
                              , NAMING_TYPE
                              , Typed_Arguments /*confirmed arguments*/
                              , Typed_Arguments /*quantified symbols*/
                              , Variables /*non-action (i.e., quantified) arguments*/
                              , Formula::Subformula /*definition*/
                              , int /*type of derived predicate*/>
    {
    public: typedef type_wrapper<ID_VAL
                                 , NAMING_TYPE
                                 , Typed_Arguments
                                 , Typed_Arguments
                                 , Variables
                                 , Formula::Subformula 
                                 , int> Parent;
        
        PRINTING;
    public:

        /* PDDL domain description of the derived predicate. */
        std::string get__description() const;

            
                
        NAMING_TYPE get__name() const ;
        Planning::Typed_Arguments get__arguments() const ;
        Planning::Typed_Arguments get__quantified_symbols() const ;
        Variables get__variables() const ;
        int get__type() const ;
        Formula::Subformula get__formula() const;
        void alter__formula(Formula::Subformula);
    };

    
    class Derived_Predicate
        : public First_Order_Derived_Symbol
        <enum_types::derived_predicate
         , Planning::Predicate_Name> {};
    
    class Derived_Percept
        : public First_Order_Derived_Symbol
        <enum_types::derived_percept
         , Planning::Percept_Name> {};
    

    typedef std::set<Derived_Predicate> Derived_Predicates;
    typedef std::set<Derived_Percept> Derived_Percepts;


}

#include "planning_derived_predicate__TEMPLATES.hh"


#endif
