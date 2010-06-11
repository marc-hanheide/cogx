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
 * CogX ::
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
#ifndef DTP_PDDL_DOMAIN_HH
#define DTP_PDDL_DOMAIN_HH


#include "dtp_pddl.hh"
#include "dtp_pddl_parsing_actions_domain.hh"

namespace Planning
{
    namespace Parsing
    {
        
        /******************************************************************************************************************
         * PDDL items that are represented by constant strings. 
         *
         * - KEYWORDS
         *
         ******************************************************************************************************************/
        
        
        struct s_Parameters : string<p, a, r, a, m, e, t, e, r, s > {}; // *** DOMAIN ELEMENT ***
        struct s_COGX_Variables : string<v, a, r, i, a, b, l, e, s > {}; // *** __COGX__ DOMAIN ELEMENT ***
        struct s_Precondition : string<p, r, e, c, o, n, d, i, t, i, o, n > {};
        struct s_Effect : string<e, f, f, e, c, t > {};
        struct s_Execution : string<e, x, e, c, u, t, i, o, n > {};

        
        
        struct s_Action : string< a, c, t, i, o, n > {};
        struct s_Derived : string<d, e, r, i, v, e, d >{}; // *** DOMAIN ELEMENT ***
        struct s_Observe : string< o,b,s,e,r,v,e > {};
        

        
        struct s_Percepts: string< p, e, r, c, e, p, t, s>{}; // *** POMDP DOMAIN ELEMENT ***
        struct s_Functions : string<f, u, n, c, t, i, o, n, s>{}; // *** DOMAIN ELEMENT ***
        struct s_Predicates : string<p, r, e, d, i, c, a, t, e, s>{}; // *** DOMAIN ELEMENT ***
        struct s_Types : string<t, y, p, e, s>{}; // *** DOMAIN ELEMENT ***
        struct s_Requirements : string<r, e, q, u, i, r, e, m, e, n, t, s>{}; // *** DOMAIN ELEMENT ***


        struct s_Number : ifapply< string<n, u, m, b, e, r>, Function_Type_Number__Action>{};
        struct s_Int : ifapply< string<i, n, t>, Function_Type_Int__Action> {};
        struct s_Double : ifapply< string<d, o, u, b, l, e>, Function_Type_Double__Action>{};

        
        /******************************************************************************************************************
         * PDDL items that are represented by alphanumeric variables strings. 
         *
         * - Typenames
         *
         * - Predicate names
         *
         * - Fluent/function names
         *
         * - Requirements names
         *
         * - Action names
         *
         * - Domain name
         *
         * - Names of objects
         *
         * - Names of constants
         * 
         ******************************************************************************************************************/

        struct Requirment_Name : ifapply<Basic_Alphanumeric, Requirement__Action> {};

        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Types (description of)
         *
         ******************************************************************************************************************/

        struct Type_Of_Type : seq<one<'-'>
                                  , pad<sor<ifapply<Either_Type, DEBUG__Action>
                                            , ifapply<Type_Name, Type_Of_Type__Action> >
                                        , space> > {};

        struct Types_Description_Element
            : ifapply<seq<Types_List, opt<Type_Of_Type> >
                      , Add_Types__Action>{};
        
        struct Types_Description
            : seq< pad<s_Types, space>
                   , ifapply<star<Types_Description_Element>
                             , Commit_Type_Hierarchy__Action> > {};

        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Predicates (description of)
         *
         ******************************************************************************************************************/
        
//         struct Type_Of_Argument : seq< one<'-'>
//                                        , pad<sor<Either_Type
//                                                  , /*SIMULATING EITHER_TYPE*/ ifapply<Type_Name, Type_Of_Type__Action>>
//                                              , space> > {};
    
        
//         struct Argument : seq< plus<sor<pad<Variable, space>
//                                         , pad<Constant_Argument, space> > >,
//                                opt<ifapply<Type_Of_Argument
//                                            ,Type_Of_Argument__Action>>>{};
        
//         struct Argument__VARIABLE_ONLY
//             : seq< plus < pad  < Variable, space> >,
//                    opt<ifapply<Type_Of_Argument
//                                ,Type_Of_Argument__Action> > >{};
        
        struct No_Parentheses_Predicate_Description : ifapply< seq<Predicate_Name
                                                                   , star<Argument> >
                                                               , Predicate_Description__Action> {};
        
        struct Predicate_Description : seq<Open
                                           , No_Parentheses_Predicate_Description
                                           , Close> {};

        struct Predicates_Description : seq<s_Predicates, star<Predicate_Description> >/*_seq*/ {};
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Perception (description of)
         *
         ******************************************************************************************************************/


        struct No_Parentheses_Percept_Description : ifapply< seq<Percept_Name
                                                                   , star<Argument> >
                                                               , Percept_Description__Action> {};
        struct Percept_Description : seq<Open
                                           , No_Parentheses_Percept_Description
                                           , Close> {};

        struct Percepts_Description : seq<s_Percepts, star<Percept_Description> >/*_seq*/ {};
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         *
         * - Requirements (description of)
         *
         * 
         ******************************************************************************************************************/
        
        struct Requirement_Name_Noise : seq< one<':'>, Requirment_Name>  {};
        
        struct Requirements_Description : seq<s_Requirements
                                              , plus<pad<Requirement_Name_Noise
                                                         , space> > >/*_seq*/ {};



        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Precondition Formulae
         *
         ******************************************************************************************************************/
        
        template<typename ATOMIC_SYMBOL>
        struct Operator : sor<ifapply<s_And, And__Action>,
                              ifapply<s_Or, Or__Action>,
                              ifapply<s_Not, Not__Action>,
                              ifapply<s_If, If__Action>,
                              ifapply<s_Forall, Forall__Action>,
                              ifapply<s_Exists, Exists__Action>,
                              
                              /*....................................*/
                              /*These arguments have to be constant.*/
                              /*....................................*/
                              ifapply<ifapply<plus<Argument__VARIABLE_ONLY>
                                              , Skip_Next____Formula__Action____Action>
                                      , Variable_Cluster__Action>,
                              /*....................................*/
                              
                              ifapply<ATOMIC_SYMBOL, Skip_Next____Formula__Action____Action>,
                              ifapply<success, Empty_Formula__Action> /* Empty formula, means no action precondition. */> {};

        template<typename ATOMIC_SYMBOL>
        struct Precondition_Subformulae
            : seq< ifapply<Open, Dive__Action>
                   , Operator<ATOMIC_SYMBOL>
                   , ifapply< star<Precondition_Subformulae<ATOMIC_SYMBOL>>,  Formula__Action>
                   , ifapply<Close, Emerge__Action> > {}; 

        struct Basic_Precondition_Subformulae : Precondition_Subformulae<Typeless_Predicate> {};
        struct Execution_Subformulae : Precondition_Subformulae<Typeless_Action> {};



        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Effect Formulae
         *
         ******************************************************************************************************************/

        
        struct Observational_Effect_Subformulae : Effect_Subformulae<Typeless_Percept> {};

        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Actions (description of)
         *
         ******************************************************************************************************************/

        
        struct Action_Header : ifapply< seq<s_Action
                                            , pad<Action_Name, space>
                                            , pad<seq<one<':'>, s_Parameters>, space>
                                            , Open
                                            , star<Argument> 
                                            , Close>
                                        , Action_Header__Action> {};
        

        struct Action_Precondition : seq< pad<seq<one<':'>, s_Precondition>, space>
                                          , Basic_Precondition_Subformulae > {};
        
        struct Action_Effect : seq< pad< seq<one<':'>, s_Effect>, space>
                                    , State_Effect_Subformulae > {};
        
        
        struct Action_Description :  ifapply<seq< Action_Header
                                                  , ifapply<Action_Precondition, Action_Precondition__Action>
                                                  , ifapply<Action_Effect, Action_Effect__Action>
                                                  >
                                             , Completed_Action__Action>{};



        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Observations (description of)
         *
         ******************************************************************************************************************/

        struct Observation_Name : ifapply<Basic_Alphanumeric , Observation_Name__Action>{};
        
        struct Observation_Header : ifapply< seq<s_Observe
                                                 , pad<Observation_Name, space>
                                                 , pad<seq<one<':'>, s_Parameters>, space>
                                                 , Open
                                                 , star<Argument> 
                                                 , Close>
                                             , Observation_Header__Action> {};
        

        struct Observation_Execution : seq< pad<seq<one<':'>, s_Execution>, space>
                                               , Execution_Subformulae > {};
        
        struct Observation_Precondition : seq< pad<seq<one<':'>, s_Precondition>, space>
                                               , Basic_Precondition_Subformulae > {};
        
        struct Observation_Effect : seq< pad< seq<one<':'>, s_Effect>, space>
                                         , Observational_Effect_Subformulae > {};
        
        
        struct Observation_Description :
            ifapply<seq< Observation_Header
                         , ifapply<Observation_Execution, Observation_Execution__Action>
                         , ifapply<Observation_Precondition, Observation_Precondition__Action>
                         , ifapply<Observation_Effect, Observation_Effect__Action> >
                    , Completed_Observation__Action>{};

        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Derived predicates (description of)
         *
         ******************************************************************************************************************/

        struct Derived_Predicate_Header : ifapply< seq<s_Derived
                                                       , Open
                                                       , pad<Predicate_Name, space>
                                                       , star<Argument> 
                                                       , Close>
                                                   , Derived_Predicate_Header__Action> {};
        
        
        struct Derived_Predicate_Description :  ifapply< seq< Derived_Predicate_Header,
                                                              Basic_Precondition_Subformulae >
                                                         , Complete_Derived_Predicate__Action > {};
                                 


                                          
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         *
         ******************************************************************************************************************/


        struct PDDL_DOMAIN_Element : sor<Types_Description
                                  , Requirements_Description
                                  , Constants_Description //:constants
                                  , Predicates_Description //:functions
                                  , Percepts_Description //:percepts
//                                   , Functions_Description //:functions
//                                   , Perceptual_Functions_Description //:s-functions
                                  , Derived_Predicate_Description //:derived
                                  , Action_Description /* :action */
                                  , Observation_Description /* :observe */
                                  > {};
        
        struct PDDL_DOMAIN_Element_Noise : seq<Open
                                        , one<':'>, PDDL_DOMAIN_Element
                                        , Close> {};
        
        struct Domain_Description : plus<PDDL_DOMAIN_Element_Noise> {};

        struct PDDL_Preamble_Domain : seq< Open
                                            , pad<s_Define, space>
                                            , Open
                                            , s_Domain
                                            , pad<Domain_Name, space>
                                            , Close
                                            , Domain_Description
                                            , Close
                                            > {};
        
    }
}

#endif
