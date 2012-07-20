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
        
        
        struct s_Parameters : stand_alone_string<p, a, r, a, m, e, t, e, r, s > {}; // *** DOMAIN ELEMENT ***
        struct s_COGX_Variables : stand_alone_string<v, a, r, i, a, b, l, e, s > {}; // *** __COGX__ DOMAIN ELEMENT ***
        struct s_Precondition : stand_alone_string<p, r, e, c, o, n, d, i, t, i, o, n > {};
        struct s_Effect : stand_alone_string<e, f, f, e, c, t > {};
        struct s_Execution : stand_alone_string<e, x, e, c, u, t, i, o, n > {};

        
        
        struct s_Action : stand_alone_string< a, c, t, i, o, n > {};
        struct s_Derived : stand_alone_string< d, e, r, i, v, e, d >{}; // *** DOMAIN ELEMENT ***
        struct s_Observe : stand_alone_string< o,b,s,e,r,v,e > {};
        

        
        struct s_Opredicates:
            stand_alone_things< string<o>
                                , one<'-'>
                                , string<p, r, e, d, i, c, a, t, e, s> > {}; // *** POMDP DOMAIN ELEMENT ***
        struct s_Spredicates:
            stand_alone_things< string<s>
                                , one<'-'>
                                , string<p, r, e, d, i, c, a, t, e, s> > {}; // *** POMDP DOMAIN ELEMENT ***
        
        struct s_Ofunctions:
            stand_alone_things< string<o>
                                , one<'-'>
                                , string<f, u, n, c, t, i, o, n, s> > {}; // *** POMDP DOMAIN ELEMENT ***
        struct s_Sfunctions:
            stand_alone_things< string<s>
                                , one<'-'>
                                , string<f, u, n, c, t, i, o, n, s> > {}; // *** POMDP DOMAIN ELEMENT ***
        struct s_Percepts: stand_alone_string< p, e, r, c, e, p, t, s> {}; // *** POMDP DOMAIN ELEMENT ***
        struct s_Functions : stand_alone_string<f, u, n, c, t, i, o, n, s> {}; // *** DOMAIN ELEMENT ***
        struct s_Predicates : stand_alone_string<p, r, e, d, i, c, a, t, e, s> {}; // *** DOMAIN ELEMENT ***

        struct Predicates_Prefix : sor<s_Predicates, s_Spredicates> {};
        struct State_Functions_Prefix : sor<s_Functions, s_Sfunctions> {};
        struct Perceptual_Functions_Prefix : s_Ofunctions {};
        struct Percepts_Prefix : sor<s_Percepts, s_Opredicates> {};
        
        
        struct s_Types : stand_alone_string<t, y, p, e, s> {}; // *** DOMAIN ELEMENT ***
        struct s_Requirements : stand_alone_string<r, e, q, u, i, r, e, m, e, n, t, s> {}; // *** DOMAIN ELEMENT ***


        struct s_Number : ifapply< stand_alone_string<n, u, m, b, e, r>, Function_Type_Number__Action>{};
        struct s_Int : ifapply< stand_alone_string<i, n, t>, Function_Type_Int__Action> {};
        struct s_Double : ifapply< stand_alone_string<d, o, u, b, l, e>, Function_Type_Double__Action>{};

        
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
        
        struct No_Parentheses_Predicate_Description : ifapply< seq<Predicate_Name
                                                                   , star<Argument> >
                                                               , Predicate_Description__Action> {};
        
        struct Predicate_Description : seq<Open
                                           , No_Parentheses_Predicate_Description
                                           , Close> {};

        struct Predicates_Description : seq<Predicates_Prefix, star<Predicate_Description> >/*_seq*/ {};
        
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

        struct Percepts_Description : seq<Percepts_Prefix, star<Percept_Description> >/*_seq*/ {};

        
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - State functions (description of)
         *
         ******************************************************************************************************************/
        
        struct No_Parentheses_State_Function_Description : ifapply< seq<State_Function_Name
                                                                        , star<Argument> >
                                                             , State_Function_Domainx_Description__Action> {};
        struct State_Function_Description : seq< Open
                                           , No_Parentheses_State_Function_Description
                                           , Close
                                           , Type_Of_Argument
                                           , ifapply<success, State_Function_Description__Action> > {};
        
        struct State_Functions_Description : seq<State_Functions_Prefix, star<State_Function_Description> >/*_seq*/ {};
        
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Perceptual functions (description of)
         *
         ******************************************************************************************************************/

        struct No_Parentheses_Perceptual_Function_Description
            : ifapply< seq<Perceptual_Function_Name
                           , star<Argument> >
                       , Perceptual_Function_Domainx_Description__Action> {};
        
        struct Perceptual_Function_Description :
            seq< Open
                 , No_Parentheses_Perceptual_Function_Description
                 , Close
                 , Type_Of_Argument
                 , ifapply<success, Perceptual_Function_Description__Action> > {};
        
        struct Perceptual_Functions_Description :
            seq<Perceptual_Functions_Prefix, star<Perceptual_Function_Description> >/*_seq*/ {};
        
        
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
        
//         template<typename ATOMIC_SYMBOL>
//         struct Precondition_Operator : sor<ifapply<s_And, And__Action>,
//                               ifapply<s_Or, Or__Action>,
//                               ifapply<s_Not, Not__Action>,
//                               ifapply<s_If, If__Action>,
//                               ifapply<s_Forall, Forall__Action>,
//                               ifapply<s_Exists, Exists__Action>,
                              
//                               /*....................................*/
//                               /*These arguments have to be constant.*/
//                               /*....................................*/
//                               ifapply<ifapply<plus<Argument__VARIABLE_ONLY>
//                                               , Skip_Next____Formula__Action____Action>
//                                       , Variable_Cluster__Action>,
//                               /*....................................*/
                              
//                               ifapply<ATOMIC_SYMBOL, Skip_Next____Formula__Action____Action>,
//                               ifapply<success, Empty_Formula__Action> /* Empty formula, means no action precondition. */> {};

//         template<typename ATOMIC_SYMBOL>
//         struct Precondition_Subformulae
//             : seq< ifapply<Open, Dive__Action>
//                    , Precondition_Operator<ATOMIC_SYMBOL>
//                    , ifapply< star<Precondition_Subformulae<ATOMIC_SYMBOL>>,  Formula__Action>
//                    , ifapply<Close, Emerge__Action> > {}; 

//         struct Basic_Precondition_Subformulae : Precondition_Subformulae<Typeless_Predicate> {};
        struct Execution_Subformulae : Precondition_Subformulae<Typeless_Action, success> {};

        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Effect Formulae
         *
         ******************************************************************************************************************/
        
        struct Observational_Effect_Subformulae : seq< ifapply<success, Start_Effect_Parsing__Action>
            , Effect_Subformulae<Typeless_Percept, Typeless_Perceptual_Function>
            , ifapply<success, Stop_Effect_Parsing__Action> >{};

        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Actions (description of)
         *
         ******************************************************************************************************************/

        
        struct Action_Header : seq<s_Action, pad<Action_Name, space> > {};
        

        struct Action_Parameters : seq< pad<seq<one<':'>, s_Parameters>, space>
                                        , Open
                                        , star<Argument> 
                                        , Close > {};
        
        struct Action_Precondition : seq< pad<seq<one<':'>, s_Precondition>, space>
                                          , Basic_Precondition_Subformulae > {};
        
        struct Action_Effect : seq< pad< seq<one<':'>, s_Effect>, space>
                                    , State_Effect_Subformulae > {};
        

        struct Action_Element : sor< Action_Parameters
                                     , ifapply<Action_Precondition, Action_Precondition__Action>
                                     , ifapply<Action_Effect, Action_Effect__Action> > {};
        
        struct Action_Description :  ifapply< ifapply<seq< Action_Header
                                                           , star<Action_Element> >
                                                      , Action_Header__Action
                                                      /*Deas with data from Action_Parameters parsing*/ >
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
        

        struct Observation_Element : sor< ifapply<Observation_Execution, Observation_Execution__Action>
                                          , ifapply<Observation_Precondition, Observation_Precondition__Action>
                                          , ifapply<Observation_Effect, Observation_Effect__Action> > {};
        
        struct Observation_Description :
            ifapply<seq< Observation_Header
                         , star<Observation_Element> >
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
                                 


        
//         /******************************************************************************************************************
//          * High level PDDL domain description elements.
//          *
//          * - Derived percepts (description of)
//          *
//          ******************************************************************************************************************/

//         struct Derived_Percept_Header : ifapply< seq<s_Derived
//                                                      , Open
//                                                      , pad<Percept_Name, space>
//                                                      , star<Argument> 
//                                                      , Close>
//                                                  , Derived_Percept_Header__Action> {};
        
        
//         struct Derived_Percept_Description :  ifapply< seq< Derived_Percept_Header,
//                                                             Basic_Precondition_Subformulae >
//                                                        , Complete_Derived_Percept__Action > {};
                                 


                                          
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         *
         ******************************************************************************************************************/


        struct PDDL_DOMAIN_Element :
            sor<Types_Description
                , Requirements_Description
                , Constants_Description //:constants
                , Predicates_Description //:predicates
                , Percepts_Description //:percepts
                , State_Functions_Description //:functions
                , Perceptual_Functions_Description //:s-functions
                , Derived_Predicate_Description //:derived
//                 , Derived_Percept_Description //:o-derived
                , Action_Description /* :action */
                , Observation_Description /* :observe */
                > {};
        
        struct PDDL_DOMAIN_Element_Noise : seq<Open
                                        , one<':'>, PDDL_DOMAIN_Element
                                        , Close> {};
        
        struct Domain_Description : plus<PDDL_DOMAIN_Element_Noise> {};


        
        struct PDDL_Preamble_Domain : // seq<one<':'>
//                                           , sor<s__Domain
//                                                 , ifapply<Basic_Alphanumeric
//                                                           , DEBUG__Action> >
//                                           , pad<s_Define, space> >
            seq< Open
                 , pad<s_Define, space>
                 , Open
                 , s_Domain
                 , pad<Domain_Name, space>
                 , Close
                 , Domain_Description
                 , Close
                 >
        {};
        
    }
}

#endif


/* Make each program do one thing well.
 *
 * -- M. D. McIlroy, E. N. Pinson, and B. A. Tague. Unix Time-Sharing
 * System Forward :: The Bell System Technical Journal :: Bell
 * Laboratories. 1978. 57 (6, part 2). p. 1902. The quoted phrase is
 * usually attributed to the first author, McIlroy.
 *
 */

