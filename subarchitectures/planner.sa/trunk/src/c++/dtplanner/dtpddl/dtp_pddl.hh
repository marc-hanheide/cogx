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
#ifndef DTP_PDDL_HH
#define DTP_PDDL_HH

#include "dtp_pddl_parsing_actions.hh"

namespace Planning
{
    namespace Parsing
    {   
        using namespace pegtl::ascii;

        /******************************************************************************************************************
         * Basic data (and string) types. 
         *
         ******************************************************************************************************************/
        struct Open : pad< one<'('>, space >{};
        
        struct Close : pad< one<')'>, space >{};

        
        struct Optional_Open : pad< sor<one<'('>, success>, space >{};
        
        struct Optional_Close : pad< sor<one<')'>, success>, space >{};
        
        struct digits : plus<digit> {};
        
        struct Number : ifapply< seq < ifapply<digits, GOT_INTEGER_NUMBER__Action>
                                       , opt< seq<one<'.'>
                                                  , ifapply<digits
                                                            , GOT_REAL_NUMBER__Action > > > >
                                 , Number__Action >
        {};
        
        /******************************************************************************************************************
         * PDDL items that are represented by constant strings. 
         *
         * - KEYWORDS
         *
         ******************************************************************************************************************/

        
        struct s_Domain : string<d, o, m, a, i, n> {};
        
        struct s_Define : string<d, e, f, i, n, e> {};
        struct s_Either : string< e, i, t, h, e, r > {};
        struct s_Constants : sor<string<c, o, n, s, t, a, n, t, s>
                                 , string<o, b, j, e, c, t, s>>{}; 
        
        struct s_And : string<a, n, d>{};
        struct s_Not : string<n, o, t>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_Or : string<o, r>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_Forall : string<f, o, r, a, l, l>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_Exists : string<e, x, i, s, t, s>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_If : string<i, f>{}; // *** COMPILED TO DISJUNCTION ***

        
        struct s_Probabilistic : string<p, r, o, b, a, b, i, l, i, s, t, i, c>{}; // *** EFFECT ***
        struct s_When : string<p, r, o, b, a, b, i, l, i, s, t, i, c>{}; // *** EFFECT ***
        struct s_Foreach : string<f, o, r, e, a, c, h>{}; // *** EFFECT ***

        
        struct s_Increase : string<i, n, c, r, e, a, s, e> {};
        struct s_Decrease : string<d, e, c, r, e, a, s, e> {};
        struct s_Assign : string<a, s, s, i, g, n> {};

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
         * - Problem name
         *
         * - Names of objects
         *
         * - Names of constants
         * 
         ******************************************************************************************************************/
        struct Basic_Alphanumeric : seq< alpha
                                         , star < sor< alpha
                                                       , digit
                                                       , one<'-'>
                                                       , one<'_'> > > >{};



        
        struct Domain_Name : ifapply<Basic_Alphanumeric , Domain_Name__Action>{};
        
        struct Predicate_Name : ifapply<Basic_Alphanumeric , Predicate_Name__Action>{};
        
        struct Function_Name : ifapply<Basic_Alphanumeric , Function_Name__Action>{};

        /* (see  \struct{Types_List} and \struct{Types_Of_Types_List} ) */
        struct Type_Name : Basic_Alphanumeric {};

        struct Action_Name : ifapply<Basic_Alphanumeric , Action_Name__Action>{};

        struct Percept_Name : ifapply<Basic_Alphanumeric , Percept_Name__Action>{};
        

        struct Constant_Name : ifapply<Basic_Alphanumeric, Constant__Action>{};
        
        
        struct Variable_Argument : ifapply<Basic_Alphanumeric, Variable_Argument__Action>{};
        
        
        struct Constant_Argument : ifapply<Basic_Alphanumeric, Constant_Argument__Action>{};
        
//         /* PDDL types as the occur in the description of predicate,
//          * fluent/function, and action arguments.*/
//         struct Type_Names : seq< Open, pad<s_Either, space>, plus<pad<Type_Name, space> >/*_plus*/, Close >/*_seq*/{};

        
        
        /******************************************************************************************************************
         * Some PDDL tokens are represented by alphanumeric strings
         * with a specific prefix. The prefix usually consists of a
         * single character. For example, a variable has the '?'
         * character as a prefix. Keywords that do not correspond to
         * formula operators (i.e., "and") often come with a ':'
         * prefix (e.g. ":action"). Essentially, this is all based
         * loosely on LISP.
         *
         * - Variable name
         * 
         ******************************************************************************************************************/
        
        struct Variable: seq< one<'?'>,  Variable_Argument>{};
        
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Types (description of)
         *
         ******************************************************************************************************************/
        struct Types_List
            : plus<pad<ifapply<Type_Name
                               , Type__Action>
                       , space> > {};
        
        struct Types_Of_Types_List
            : plus<pad<ifapply<Type_Name
                               , Type_Of_Type__Action>
                       , space> > {}; 
        
        struct Either_Type : seq<Open
                                 , pad<s_Either, space>
                                 , Types_Of_Types_List
                                 , Close>{};
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Predicates (description of)
         *
         ******************************************************************************************************************/
        
        struct Type_Of_Argument : seq< one<'-'>
                                       , pad<sor<Either_Type
                                                 , /*SIMULATING EITHER_TYPE*/ ifapply<Type_Name, Type_Of_Type__Action>>
                                             , space> > {};
    
        
        struct Argument : seq< plus<sor<pad<Variable, space>
                                        , pad<Constant_Argument, space> > >,
                               opt<ifapply<Type_Of_Argument
                                           ,Type_Of_Argument__Action>>>{};

        
        struct Argument__VARIABLE_ONLY
            : seq< plus < pad  < Variable, space> >,
                   opt<ifapply<Type_Of_Argument
                               ,Type_Of_Argument__Action> > >{};
        
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Constants (description of)
         *
         ******************************************************************************************************************/
        struct Constants_List : plus<pad<ifapply<Constant_Name
                                                 , Constant__Action>
                                         , space> > {};
        
        struct Type_Of_Constant : seq<one<'-'>
                                      , pad< sor<ifapply<Either_Type
                                                         , Type_Of_Type__TO__Type_Of_Constant__Action> 
                                                 , ifapply<Type_Name
                                                           , Type_Of_Constant__Action > >
                                             , space>
                                      >
        {};
        
        struct Constants_Description_Element
            : ifapply<seq<Constants_List, opt<Type_Of_Constant> >
                      , Add_Constants__Action>{};
        
        struct Constants_Description :  seq<s_Constants,
                                            ifapply<star<Constants_Description_Element>
                                                    , Commit_Constants__Action> >/*_seq*/{} ;
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * -  Formulae
         *
         ******************************************************************************************************************/
        
        struct Typeless_Predicate : ifapply<seq<Predicate_Name, star<Argument> >, Formula_Predicate__Action> {};
        struct Typeless_Percept : ifapply<seq<Percept_Name, star<Argument> >, Formula_Percept__Action> {};
        struct Typeless_Action : ifapply<seq<Action_Name, star<Argument> >, Formula_Action__Action> {};
        struct Typeless_Function : ifapply<seq<Function_Name, star<Argument> >, Formula_Function__Action> {};


        /* TODO FIX HERE -- Bollocks, because there is no prefix to
         * distinguish predicates from functions. This has to be
         * repaired from the context (see
         * \module{dtp_pddl_parsing_data_formula}). */
        struct Typeless_Predicate_or_Function : sor<Typeless_Predicate, Typeless_Function> {};
        

        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Effect Formulae
         *
         ******************************************************************************************************************/

        template<typename ATOMIC_SYMBOL>
        struct Effect_Subformulae;

        template<typename ATOMIC_SYMBOL>
        struct Effect_Operator : sor<
            ifapply<s_And, And__Action>
            , ifapply<s_Not, Not__Action>
            , ifapply<s_Increase, Increase__Action>
            , ifapply<s_Decrease, Decrease__Action>
            , ifapply<s_Assign, Assign__Action>
            , ifapply<s_Probabilistic, Probabilistic__Action>/* Derived action */
            , ifapply<s_Forall, Forall_Effect__Action>/* Derived action */
            , ifapply<s_Foreach, Forall_Effect__Action>/* Derived action */
            , ifapply<s_When, Conditional_Effect__Action>/* Derived action */
            
        /*....................................*/
        /*These arguments have to be constant.*/
        /*....................................*/
            , ifapply<ifapply<plus<Argument__VARIABLE_ONLY>
                              , Skip_Next____Formula__Action____Action>
                      , Variable_Cluster__Action>
        /*....................................*/
            
            , ifapply<ATOMIC_SYMBOL, Skip_Next____Formula__Action____Action> // e.g. Typeless_Predicate or Typeless_Percept
            , ifapply<success, Empty_Formula__Action> /* Empty formula, means no action effect. */
        > {};

        
        template<typename ATOMIC_SYMBOL>
        struct Effect_Subformulae_With_Parenthesis
            : seq< ifapply<Open, Dive__Action>
                   , Effect_Operator<ATOMIC_SYMBOL>
                   , ifapply< star<Effect_Subformulae<ATOMIC_SYMBOL> >,  Formula__Action>
                   , ifapply<Close, Emerge__Action> > {};
        
        struct Effect_Subformulae_Without_Parenthesis
            : ifapply<pad<Number, space>, Number_In_Effect__Action> {};
        
        template<typename ATOMIC_SYMBOL>
        struct Effect_Subformulae : sor< Effect_Subformulae_Without_Parenthesis
                                        , Effect_Subformulae_With_Parenthesis<ATOMIC_SYMBOL> >
        {};
            
        struct State_Effect_Subformulae : Effect_Subformulae<Typeless_Predicate_or_Function> {};//Typeless_Predicate> {};
        
        //////////////////////////////////////////////////////////////////////////////
        //////////////////////////////TESTING        /////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////
//         struct Domain_Description :
//             seq< Predicates_Description, space_until_eof>{};
//         struct Domain_Description :
//             seq< Constants_Description, space_until_eof>{};
//         struct Domain_Description :
//             seq< Requirements_Description, space_until_eof>{};
//         struct Domain_Description :
//             seq< Functions_Description, space_until_eof>{};
//         struct Domain_Description :
//             seq< Action_Description, space_until_eof>{};
//         struct Domain_Description :
//             seq< Types_Description, space_until_eof>{};
    }
}

#endif
