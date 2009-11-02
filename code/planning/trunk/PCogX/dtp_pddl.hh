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
        
        struct digits : plus<digit> {};
        
        struct Number : ifapply< seq<digits, one<'.'>, digits>/*_seq*/,
                                 Number__Action>/*_ifapply*/  {};
        
        /******************************************************************************************************************
         * PDDL items that are represented by constant strings. 
         *
         * - KEYWORDS
         *
         ******************************************************************************************************************/
        struct s_Either : string< e, i, t, h, e, r > {};
        struct s_Action : string< a, c, t, i, o, n > {};
        struct s_Perception : string< p, e, r, c, e, p, t, i, o, n>{};
        struct s_Parameters : string<p, a, r, a, m, e, t, e, r, s > {};
        struct s_Precondition : string<p, r, e, c, o, n, d, i, t, i, o, n > {};
        struct s_Effect : string<e, f, f, e, c, t > {};
        struct s_Execution : string<e, x, e, c, u, t, i, o, n > {};
        
        struct s_Define : string<d, e, f, i, n, e> {};
        struct s_Problem : string<p, r, o, b, l, e, m> {};
        struct s_Domain : string<d, o, m, a, i, n> {};
        
        struct s_Increase : string<i, n, c, r, e, a, s, e> {};
        struct s_Decrease : string<d, e, c, r, e, a, s, e> {};
        
        struct s_And : string<a, n, d>{};
        struct s_Not : string<n, o, t>{};
        struct s_Or : string<o, r>{};
        struct s_Forall : string<f, o, r, a, l, l>{};
        struct s_Exists : string<e, x, i, s, t, s>{};
        struct s_If : string<i, f>{};
        struct s_Probabilistic : string<p, r, o, b, a, b, i, l, i, s, t, i, c>{};
        struct s_Foreach : string<f, o, r, e, a, c, h>{};
        
        struct s_Functions : string<f, u, n, c, t, i, o, n, s>{};
        struct s_Predicates : string<p, r, e, d, i, c, a, t, e, s>{};
        struct s_Types : string<t, y, p, e, s>{};
        struct s_Constants : string<c, o, n, s, t, a, n, t, s>{};
        struct s_Requirements : string<r, e, q, u, i, r, e, m, e, n, t, s>{};
        
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
                                                       , one<'-'> > > >{};
        
        struct Domain_Name : ifapply<Basic_Alphanumeric , Domain_Name__Action>{};
        
        struct Predicate_Name : ifapply<Basic_Alphanumeric , Predicate_Name__Action>{};
        
        struct Function_Name : ifapply<Basic_Alphanumeric , Function_Name__Action>{};
        
        struct Type_Name : ifapply<Basic_Alphanumeric , Type__Action>{};
        
        struct Action_Name : ifapply<Basic_Alphanumeric , Action_Name__Action>{};

        struct Perception_Name : ifapply<Basic_Alphanumeric , Perception_Name__Action>{};

        struct Constant_Name : ifapply<Basic_Alphanumeric, Constant__Action>{};
        
        struct Requirment_Name : ifapply<Basic_Alphanumeric, Requirement__Action> {};

        struct Variable_Name : ifapply<Basic_Alphanumeric, Variable__Action>{};
        
        /* PDDL types as the occur in the description of predicate,
         * fluent/function, and action arguments.*/
        struct Type_Names : seq< Open, pad<s_Either, space>, plus<pad<Type_Name, space> >/*_plus*/, Close >/*_seq*/{};

        struct Requirement_Name_Noise : seq< one<':'>, Requirment_Name>  {};
        
        
        /******************************************************************************************************************
         * PDDL items that are represented by alphanumeric variables
         * strings with a specific prefix. The prefix usually consists
         * of a single character. For example, a variable has the '?'
         * character as a prefix.
         *
         * - Variable name
         * 
         ******************************************************************************************************************/
        
        struct Variable: seq< one<'?'>,  Variable_Name>{};

        
        /******************************************************************************************************************
         * PDDL items that have a list representation. Action,
         * predicate, and functions are described in terms of their
         * unique names, and also a "list" of (typed) argument
         * variables. In the author's view it is not clear that the
         * initial intention behind the design of PDDL was to have
         * _lists_ (resp. unordered containers) for description of
         * n-ary term arguments. However, whatever the original
         * intention, other folk tend to treat these things as
         * ordered.
         *
         * - List of variables
         *
         * - List of types
         *
         ******************************************************************************************************************/
        
        struct List_of_Variables : plus< pad <Variable, space > >{};

        /* Describing the type[/s] of a variable.*/
        struct Type_Description : seq < pad<one<'-'>, space>, sor<Type_Name, Type_Names> > {};        
        
        /******************************************************************************************************************
         * Description of either: an action, predicate, or function arguments.
         ******************************************************************************************************************/


        struct Variable_Arguments : ifapply< seq<List_of_Variables, opt<Type_Description>/*_opt*/>/*_seq*/
                                             , Build_Argument__Action>/*_ifapply*/{};
        
        struct Constant_Arguments : ifapply< plus<pad<Constant_Name, space> >/*_plus*/
                                             , Build_Argument__Action>/*_ifapply*/ {};
        
        struct Arguments
            : plus< sor<Variable_Arguments, Constant_Arguments>/*_sor*/ >/*_plus*/
        {};

        struct No_Parentheses_Predicate_Description : ifapply< seq<pad<Predicate_Name, space>
                                                                   , opt<Arguments> >
                                                               , Predicate_Description__Action> {};
        
        struct Predicate_Description : seq<Open
                                           , No_Parentheses_Predicate_Description
                                           , Close> {};

        struct No_Parentheses_Function : seq< pad<Function_Name, space>
                                              , opt<Arguments> >  {};

        struct Function : seq<Open
                              , No_Parentheses_Function
                              , Close> {};
        
        struct Function_Description_With_Type : ifapply< seq< Function
                                                              , pad<one<'-'>, space>
                                                              , sor<s_Int, s_Double, s_Number> >
                                                         , Function_Description__Action> {};
        
        /******************************************************************************************************************
         * Action effect description 
         *
         ******************************************************************************************************************/

        struct Effect;

        
        struct Quantified_Probabilistic_Effect : seq<pad<s_Foreach, space>,
                                                     Predicate_Description,
                                                     Effect>{};

        struct Enumerated_Probabilistic_Effect : plus< seq<pad<Number, space>, Effect> > {} ;
        
        struct Probabilistic_Effect : seq<pad<s_Probabilistic, space>,
                                          sor<Quantified_Probabilistic_Effect,
                                              Enumerated_Probabilistic_Effect> >{};

        struct Delete_Effect : seq< pad<s_Not, space>
                                    , ifapply<Predicate_Description
                                              , Negative_Effect__Action > >{};
        
        struct Conjunctive_Effect : seq< pad<s_And, space>
                                         , plus<Effect> > {};

        struct Function_Effect : seq< ifapply<Function, Function__Action> 
                                     , pad<Number, space> > {};
        
        struct Increase_Effect : seq<pad<s_Increase, space>
                                     , Function_Effect> {};
        
        struct Decrease_Effect : seq<pad<s_Decrease, space>
                                     , Function_Effect> {};

        struct No_Parentheses_Effect : sor<Probabilistic_Effect
                                           , Conjunctive_Effect
                                           , Delete_Effect
                                           , Increase_Effect
                                           , Decrease_Effect
                                           , No_Parentheses_Predicate_Description> {};
        
        struct Effect : seq< Open,
                             No_Parentheses_Effect,
                             Close> {};
        
        
        /******************************************************************************************************************
         * Precondition description
         *
         ******************************************************************************************************************/
        
        struct Precondition;
        
        struct Not_Condition : seq<pad<s_Not, space>
                                   , ifapply<Predicate_Description, Not_Precondition__Action> > {};
        
        struct And_Condition : seq<pad<s_And, space>
                                   , plus<Precondition> > {};

        struct No_Parentheses_Precondition : sor<And_Condition
                                                 , Not_Condition
                                                 , ifapply<No_Parentheses_Predicate_Description
                                                           , Predicate_Precondition__Action> > {};
        
        struct Precondition : seq< Open,
                                   No_Parentheses_Precondition,
                                   Close> {};
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         * - Types (description of)
         *
         * - Constants (description of)
         *
         * - Predicates (description of)
         *
         * - Requirements (description of)
         *
         * - Functions (description of)
         *
         * - Actions (description of)
         *
         * - Observations (description of)
         *
         ******************************************************************************************************************/
        /* Describing the types[/s] available in a domain. UNIMPLEMENTED*/
        struct Types_Description :  seq<s_Types, ifapply<plus<pad<Type_Name, space> >, Types__Action> >/*_seq*/{} ;
        
        struct Predicates_Description : seq<s_Predicates, ifapply<plus<Predicate_Description>,
                                                                  Domain_Predicates__Action> >/*_seq*/ {};
        
        struct Constants_Description :  seq<s_Constants, ifapply< plus<Constant_Name>, Domain_Constants__Action> >/*_seq*/{} ;
        
        struct Requirements_Description : seq<s_Requirements
                                              , plus<pad<Requirement_Name_Noise
                                                         , space> > >/*_seq*/ {};
        
        struct Functions_Description :  seq<s_Functions,
                                            ifapply<plus<Function_Description_With_Type>,
                                                    Domain_Functions__Action> >/*_seq*/{} ;
        
        struct Action_Description :  ifapply< seq<s_Action,
                                                  ifapply<seq<pad<Action_Name, space>,
                                                              pad<seq<one<':'>, s_Parameters>, space>,
                                                              Open,
                                                              Arguments,
                                                              Close >, Action_Signature__Action > ,
                                                  pad<seq<one<':'>, s_Precondition>, space>,
                                                  Precondition
                                                  , pad<seq<one<':'>, s_Effect>, space>
                                                  , Effect >/*_seq*/
                                              , Make_Action__Action> {} ;

        struct Required_Action : ifapply<Predicate_Description
                                         , Perception_Action__Action> {};
        
        struct Perception_Description :  ifapply< seq<s_Perception
                                                      , ifapply<seq<pad<Perception_Name, space>
                                                                    , pad<seq<one<':'>, s_Parameters>, space>
                                                                    , Open
                                                                    , Arguments
                                                                    , Close >
                                                                , Perception_Signature__Action > 
                                                      , pad<seq<one<':'>, s_Execution>, space>
                                                      , Required_Action
                                                      , pad<seq<one<':'>, s_Precondition>, space>
                                                      , Precondition
                                                      , pad<seq<one<':'>, s_Effect>, space>
                                                      , Effect>/*_seq*/
                                                  , Make_Perception__Action> {} ;
            
        struct PDDL_Element : sor<Types_Description
                                  , Requirements_Description
                                  , Constants_Description
                                  , Action_Description
                                  , Perception_Description
                                  , Predicates_Description
                                  , Functions_Description> {};
        
        
        //seq<s_Types, plus<Type_Name> >
        struct PDDL_Element_Noise : seq<Open, one<':'>, PDDL_Element, Close >{};

        struct Problem_Description : string<u, n, i, m, p, l, e, m, e, n, t, e, d> /*_UNIMPLEMENTED*/{};
        
        
        struct Domain_Description : plus<PDDL_Element_Noise> {};
        
        struct PDDL_Problem : seq<s_Problem, Close, Problem_Description>{};
        
        struct PDDL_Domain : seq<s_Domain
                                 , pad<Domain_Name, space>
                                 , Close
                                 , Domain_Description>{};
        
        struct PDDL_Preamble_Problem : seq<Open
                                           , s_Define
                                           , Open/*_Closed elsewhere*/
                                           , PDDL_Problem
                                           , Close> {};
        
        struct PDDL_Preamble_Domain : seq<Open
                                          , s_Define
                                          , Open/*_Closed elsewhere*/
                                          , PDDL_Domain
                                          , Close> {};
        
        struct PDDL_Preamble : seq<Open
                                   , s_Define
                                   , Open/*_Closed elsewhere*/
                                   , sor<PDDL_Problem, PDDL_Domain>
                                   , Close> {};
        
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
