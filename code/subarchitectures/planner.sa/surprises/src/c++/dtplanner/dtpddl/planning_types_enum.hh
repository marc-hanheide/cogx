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
#ifndef PLANNING_TYPES_ENUM_HH
#define PLANNING_TYPES_ENUM_HH


namespace Planning
{
    enum enum_types {
        /* Formula types*/
        negation = 0,
        state_predicate = 1,
        state_proposition = 2,         /* PDDL "proposition". So a ground predicate symbol.*/
        conjunction = 3,
        disjunction = 4,
        material_implication = 5,
        exists = 6,
        forall = 7,
        vacuous = 8,
        formula_true = 9,
        formula_false = 10,
        observational_predicate = 11,
        observational_proposition = 12,         /* DT-PDDL "ground observational schema".*/
        perceptual_predicate = 13,
        perceptual_proposition = 14,         /* DT-PDDL "ground percept". So a ground percept symbol.*/
        state_ground_function,
        state_function,
        perceptual_ground_function,
        perceptual_function,
        action_predicate,           /* DT-PDDL "last action executed". Used in (:observe) schema.*/
        action_proposition,         /* DT-PDDL "last action executed". Used in (:observe) schema.*/
        /*extra formula elements for probabilistic effects.*/
        number,
        probabilistic_effect,
        conditional_effect,
        increase, /* PDDL function evaluation modification. */
        decrease, /* PDDL function evaluation modification. */
        assign,   /* PDDL function evaluation modification. */
        equality_test,  /* PDDL equality testing predicate. */
        /**/
        scope,      /* a variable must have a scope For example, it could be
                     * a predicate argument.*/
        type,       /* PDDL "type". Representation should be \class{std::string} based.*/
        variable,   /* PDDL "variable". Representation should be \class{std::string} based.*/
        constant,   /* PDDL "constant" (also "object"). Representation should be \class{std::string} based.*/
        requirement,/* PDDL requirement string. Representation should be \class{std::string} based.*/
        predicate_name,/* Name of a PDDL "predicate". (as above \class{string} based)*/
        percept_name,/* Name of a DT-PDDL "percept". (as above \class{string} based)*/
        state_function_name,/* Name of a PDDL "function". (as above \class{string} based)*/
        perceptual_function_name,/* Name of a DT-PDDL "perceptual function". (as above \class{string} based)*/
        domain_name,/* Name of a PDDL "domain". (as above \class{string} based)*/
        problem_name,/* Name of a PDDL "problem". (as above \class{string} based)*/
        predicate_description,     /* PDDL "predicate".*/
        state_function_description,     /* PDDL "function".*/
        percept_description, /* DT-PDDL "perceptive predicate".*/
        perceptual_function_description, /* DT-PDDL "perceptive predicate".*/
        action_name,   /* Name of a PDDL "action". (as above \class{string} based)*/
        action_header, /*(action_name ?x1 - t1 x2 - t1 etc....)*/
        observation_name,   /* Name of a DT-PDDL ":observe". (as above \class{string} based)*/
        observation_header, /*(observation_name ?x1 - t1 x2 - t1 etc....)*/
        derived_predicate_header, /*(predicate_name ?x1 - t1 x2 - t1 etc....)*/
        derived_predicate, /*(:derived )*/
        derived_percept_header, /*(predicate_name ?x1 - t1 x2 - t1 etc....)*/
        derived_percept, /*(:derived )*/
        action_schema, /* PDDL "operator" -- i.e., from domain description.*/
        observation_schema, /* DT-PDDL "observational operator" -- i.e., from domain description.*/
        /* ------------------------ NON-PARSING TYPES ------------------------ */
        /* ------------------------ NON-PARSING TYPES ------------------------ */
        /* ------------------------ NON-PARSING TYPES ------------------------ */
        /* ------------------------ NON-PARSING TYPES ------------------------ */
        /* ------------------------ NON-PARSING TYPES ------------------------ */
        literal,
        disjunctive_clause,
        conjunctive_normal_form_formula,
        ground__action,      /* PDDL "action" -- e.g., ground operator symbol.*/
        /* ------------------------ State Transformations and actions ------------------------ */
        /* ------------------------ State Transformations and actions ------------------------ */
        /* ------------------------ State Transformations and actions ------------------------ */
        /* ------------------------ State Transformations and actions ------------------------ */
        /* ------------------------ State Transformations and actions ------------------------ */
        state_transformation,
        probabilistic_state_transformation,
        simple_int_transformation,
        simple_double_transformation,
        /* ------------------------ Observations ------------------------ */
        /* ------------------------ Observations ------------------------ */
        /* ------------------------ Observations ------------------------ */
        /* ------------------------ Observations ------------------------ */
        /* ------------------------ Observations ------------------------ */
        action_literal,
        action_disjunctive_clause,
        action_conjunctive_normal_form_formula,
        observation,
        probabilistic_observation
//         int_valued_function, /* PDDL integer valued function symbol --
//                               * e.g. keyword "total-cost" for
//                               * propositional planning problems with
//                               * action costs.*/
//         real_valued_function, /* PDDL real valued function symbol. In
//                                * PPDDL we have that the probability of
//                                * an action outcome occurring ---when
//                                * the stochastic action is executed---
//                                * can be given in terms of a real
//                                * number between 0 and 1.*/
//         object_valued_function /* Something Freiburg folk are
//                                 * particularly enamoured by. Started
//                                 * with Bäckström and never really
//                                 * stopped. It's a function symbol that
//                                 * takes arguments in the
//                                 * object(/constants) space of the
//                                 * problem at hand, and then has a
//                                 * range in the same space.
//                                 * Ingenious.. really!*/
    };
}


#endif
