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
        
        struct Equal : pad< one<'='>, space >{};

        
        struct Optional_Open : pad< sor<one<'('>, success>, space >{};
        
        struct Optional_Close : pad< sor<one<')'>, success>, space >{};
        
        struct digits : plus<digit> {};
        
        struct Number : ifapply< seq < opt<one<'-'>>, ifapply<digits, GOT_INTEGER_NUMBER__Action>
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

        
        /*PDDL items completion*/
        struct pddl_item_completion :
            sor< space
                 , one<')'>
                 , one<'('> > {}; 
    
        /*A string that is not the prefix for a user-defined symbol, but rather a keyword.*/
//         template<int... Chars >
//         struct stand_alone_string : ifthen<string<Chars...>, not_at<pddl_item_completion>>{};
        template<int... Chars >
        struct stand_alone_string : seq<string<Chars...>, at<pddl_item_completion> >{};
        
        template<typename...  Things>
        struct stand_alone_things : seq<Things..., at<pddl_item_completion> >{};
        
        struct s_Domain : stand_alone_string<d, o, m, a, i, n> {};
        
//         struct s_Define : string<d, e, f, i, n, e> {};
        struct s_Define : stand_alone_string<d, e, f, i, n, e> {};
        struct s_Either : stand_alone_string< e, i, t, h, e, r > {};
        struct s_Constants : sor<stand_alone_string<c, o, n, s, t, a, n, t, s>
                                 , stand_alone_string<o, b, j, e, c, t, s>>{}; 
        
        struct s_And : sor<stand_alone_string<a, n, d>, stand_alone_string<A, N, D> >{};
        struct s_Not : stand_alone_string<n, o, t>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_Or : stand_alone_string<o, r>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_Forall : stand_alone_string<f, o, r, a, l, l>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_Exists : stand_alone_string<e, x, i, s, t, s>{}; // *** Will generate DERIVED PREDICATE *** 
        struct s_If : stand_alone_string<i, f>{}; // *** COMPILED TO DISJUNCTION ***

        
        struct s_Probabilistic : stand_alone_string<p, r, o, b, a, b, i, l, i, s, t, i, c>{}; // *** EFFECT ***
        struct s_When : stand_alone_string<w, h, e, n>{}; // *** EFFECT ***
        struct s_Foreach : stand_alone_string<f, o, r, e, a, c, h>{}; // *** EFFECT ***

        
        struct s_Increase : stand_alone_string<i, n, c, r, e, a, s, e> {};
        struct s_Decrease : stand_alone_string<d, e, c, r, e, a, s, e> {};
        struct s_Assign : stand_alone_string<a, s, s, i, g, n> {};

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
        
        struct State_Function_Name : ifapply<Basic_Alphanumeric , State_Function_Name__Action>{};

        /* (see  \struct{Types_List} and \struct{Types_Of_Types_List} ) */
        struct Type_Name : Basic_Alphanumeric {};

        struct Action_Name : ifapply<Basic_Alphanumeric , Action_Name__Action>{};

        
        struct Percept_Name : ifapply<Basic_Alphanumeric , Percept_Name__Action>{};
        
        struct Perceptual_Function_Name : ifapply<Basic_Alphanumeric , Perceptual_Function_Name__Action>{};
        

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
                                           , Type_Of_Argument__Action>>>{};

        
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
        struct Typeless_Function : ifapply<seq<State_Function_Name, star<Argument> >, Formula_State_Function__Action> {};
        struct Typeless_Perceptual_Function : ifapply<seq<Perceptual_Function_Name, star<Argument> >, Formula_Perceptual_Function__Action> {};

        /* TODO FIX HERE -- Bollocks, because there is no prefix to
         * distinguish predicates from functions. This has to be
         * repaired from the context (see
         * \module{dtp_pddl_parsing_data_formula}). */
        struct Typeless_Predicate_or_Function : sor<Typeless_Predicate, Typeless_Function> {};
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Precondition Formulae
         *
         ******************************************************************************************************************/

        struct Subformulae_Without_Parenthesis
            : sor< ifapply<pad<Number, space>, Number_In_Formula__Action>
                   , ifapply<Variable, Object_In_Formula__Action>
                   , ifapply<Constant_Argument, Constant_In_Formula__Action> >{};
        
        template<typename ATOMIC_FUNCTION_SYMBOL>
        struct Equality_Term : sor< Subformulae_Without_Parenthesis
                                    , seq< ifapply<Open, Dive__Action>
                                          , ATOMIC_FUNCTION_SYMBOL
                                          , ifapply<Close, Emerge__Action>>
                                    >{};
        
        template<typename ATOMIC_FUNCTION_SYMBOL>
        struct Equality_Test : seq< ifapply<one<'='>, Equality__Action>
                                    , Equality_Term< ATOMIC_FUNCTION_SYMBOL>
                                    , Equality_Term< ATOMIC_FUNCTION_SYMBOL> > {};
        
        template<typename ATOMIC_PREDICATE_SYMBOL, typename ATOMIC_FUNCTION_SYMBOL>
        struct Precondition_Operator :
            sor< ifapply<s_And, And__Action>
                 , ifapply<s_Or, Or__Action>
                 , ifapply<s_Not, Not__Action>
                 , ifapply<s_If, If__Action>
                 , ifapply<s_Forall, Forall__Action>
                 , ifapply<s_Exists, Exists__Action>
            
                 , Equality_Test<ATOMIC_FUNCTION_SYMBOL>
            /*....................................*/
            /*These arguments have to be constant.*/
            /*....................................*/
                 , ifapply<ifapply<plus<Argument__VARIABLE_ONLY>
                                   , Skip_Next____Formula__Action____Action>
                           , Variable_Cluster__Action>
            /*....................................*/
                
                 , ifapply<ATOMIC_PREDICATE_SYMBOL, Skip_Next____Formula__Action____Action>
                 , ifapply<success, Empty_Formula__Action> /* Empty formula, means no action precondition. */> {};
        
        template<typename ATOMIC_PREDICATE_SYMBOL, typename ATOMIC_FUNCTION_SYMBOL>
        struct Precondition_Subformulae
            : seq< ifapply<Open, Dive__Action>
                   , Precondition_Operator<ATOMIC_PREDICATE_SYMBOL, ATOMIC_FUNCTION_SYMBOL>
                   , ifapply< star<Precondition_Subformulae<ATOMIC_PREDICATE_SYMBOL
                                                            ,  ATOMIC_FUNCTION_SYMBOL> >
                              ,  Formula__Action>
                   , ifapply<Close, Emerge__Action> > {}; 

        struct Basic_Precondition_Subformulae
            : Precondition_Subformulae<Typeless_Predicate
                                       , Typeless_Function> {};
        

        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Effect Formulae
         *
         ******************************************************************************************************************/

        template<typename ATOMIC_PREDICATE_SYMBOL, typename ATOMIC_FUNCTION_SYMBOL>
        struct Effect_Subformulae;

        template<typename ASSIGNMENT_TYPE, typename ASSIGNMENT_ACTION, typename ATOMIC_FUNCTION_SYMBOL>
        struct Numerical_Formula :
            seq< ifapply<ASSIGNMENT_TYPE, ASSIGNMENT_ACTION>
                 , ifapply<Open, Dive__Action>, Typeless_Function, ifapply<Close, Emerge__Action>
                 , sor< Subformulae_Without_Parenthesis
                       , seq< ifapply<Open, Dive__Action>
                              , ATOMIC_FUNCTION_SYMBOL
                              , ifapply<Close, Emerge__Action>
                   >>
                 >
            
        {};
        
        template<typename ATOMIC_PREDICATE_SYMBOL, typename ATOMIC_FUNCTION_SYMBOL>
        struct Effect_Operator : sor<
            ifapply<s_And, And__Action>
            , ifapply<s_Not, Not__Action>
            , Numerical_Formula<s_Increase, Increase__Action, ATOMIC_FUNCTION_SYMBOL>
            , Numerical_Formula<s_Decrease, Decrease__Action, ATOMIC_FUNCTION_SYMBOL>
            , Numerical_Formula<s_Assign, Assign__Action, ATOMIC_FUNCTION_SYMBOL>

            /* Equality testing can occur in an effect formula where we admit conditional effects.*/
            , Equality_Test<ATOMIC_FUNCTION_SYMBOL>
            
            , ifapply<s_Probabilistic, Probabilistic__Action>/* Derived action */
            , ifapply<s_Forall, Forall_Effect__Action>/* Derived action */
            , ifapply<s_Foreach, Forall_Effect__Action>/* Derived action */
            , seq< ifapply<s_When, Conditional_Effect__Action>
                   , Basic_Precondition_Subformulae
                   , Effect_Subformulae<ATOMIC_PREDICATE_SYMBOL, ATOMIC_FUNCTION_SYMBOL> >/* Derived action */
            
        /*........................................*/
        /*These arguments have to be non-constant.*/
        /*........................................*/
            , ifapply<ifapply<plus<Argument__VARIABLE_ONLY>
                              , Skip_Next____Formula__Action____Action>
                      , Variable_Cluster__Action>
        /*....................................*/
            
            , ifapply<ATOMIC_PREDICATE_SYMBOL, Skip_Next____Formula__Action____Action> // e.g. Typeless_Predicate or Typeless_Percept
            , ifapply<success, Empty_Formula__Action> /* Empty formula, means no action effect. */
        > {};

        
        
        template<typename ATOMIC_PREDICATE_SYMBOL, typename ATOMIC_FUNCTION_SYMBOL>
        struct Effect_Subformulae_With_Parenthesis
            : seq< ifapply<Open, Dive__Action>
                   , Effect_Operator<ATOMIC_PREDICATE_SYMBOL, ATOMIC_FUNCTION_SYMBOL>
                   , ifapply< star<Effect_Subformulae<ATOMIC_PREDICATE_SYMBOL
                                                      , ATOMIC_FUNCTION_SYMBOL> >
                              ,  Formula__Action>
                   , ifapply<Close, Emerge__Action> > {};
        
        
        template<typename ATOMIC_PREDICATE_SYMBOL, typename ATOMIC_FUNCTION_SYMBOL>
        struct Effect_Subformulae : sor< Subformulae_Without_Parenthesis
                                        , Effect_Subformulae_With_Parenthesis<ATOMIC_PREDICATE_SYMBOL, ATOMIC_FUNCTION_SYMBOL> >
        {};
            
        struct State_Effect_Subformulae : seq<  ifapply<success, Start_Effect_Parsing__Action>
        , Effect_Subformulae<Typeless_Predicate, Typeless_Function>
        ,  ifapply<success, Stop_Effect_Parsing__Action> >{};//Typeless_Predicate> {};
    }
}

#endif

/* Worse is better.
 *
 *  -- Richard P. Gabriel
 *
 */
