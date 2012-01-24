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

#ifndef DTP_PDDL_PROBLEM_HH
#define DTP_PDDL_PROBLEM_HH


#include "dtp_pddl.hh"
#include "dtp_pddl_parsing_actions_problem.hh"

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


        struct s_Problem : stand_alone_string<p, r, o, b, l, e, m> {};
        struct s_Init : sor<
            stand_alone_string<i, n, i, t>,
            stand_alone_string<I, N, I, T> >{};
        
        struct s_Goal : stand_alone_string<g, o, a, l> {};
        
        struct s_Metric : stand_alone_string<m, e, t, r, i, c> {};
        struct s_Maximise : stand_alone_string<m, a, x, i, m, i, s, e> {};
        struct s_Minimise : stand_alone_string<m, i, n, i, m, i, s, e> {};
        struct s_Maximize : stand_alone_string<m, a, x, i, m, i, z, e> {};
        struct s_Minimize : stand_alone_string<m, i, n, i, m, i, z, e> {};
        
        /******************************************************************************************************************
         * PDDL items that are represented by alphanumeric variables strings. 
         *
         * - Typenames
         *
         * - Predicate names
         *
         * - Problem name
         *
         * - Names of objects and constants
         * 
         ******************************************************************************************************************/

        
        struct Problem_Name : ifapply<Basic_Alphanumeric , Problem_Name__Action>{};
                     
        /******************************************************************************************************************
         * Initial states
         *
         *
         ******************************************************************************************************************/
        
        struct Initial_State
            : seq< pad<s_Init, space>
                   , ifapply<success, Dive__Action>
                   , ifapply<success, Start_Initial_State_Parsing__Action>
                   , ifapply<success, And__Action>
                   , star< Effect_Subformulae<Typeless_Predicate, Typeless_Function> >
                   , ifapply<success, Formula__Action>
                   , ifapply<success, Emerge__Action>
                   , ifapply<success, Starting_State__Action>
                   , ifapply<success, Stop_Initial_State_Parsing__Action> > {};
        
        /******************************************************************************************************************
         * Metrics.
         *
         *
         ******************************************************************************************************************/

        struct Maximise : ifapply<sor<s_Maximise, s_Maximize>, Maximise__Action> {};
        struct Minimise : ifapply<sor<s_Minimise, s_Minimize>, Minimise__Action> {};
        struct Optimisation_Criteria : sor<Maximise
                                           , Minimise> {};
        
        struct Metric
            : seq< pad<s_Metric, space>
                   , Optimisation_Criteria
                   , ifapply<Open, Dive__Action>
                   , Typeless_Function
                   , ifapply<Close, Emerge__Action> 
                   , ifapply<success, Objective_Formula__Action>  > {};


        /******************************************************************************************************************
         * Goals.
         *
         *
         ******************************************************************************************************************/

        struct Goal
            : seq< pad<s_Goal, space>
                   , Basic_Precondition_Subformulae
                   , ifapply<success, Goal_Formula__Action>  > {};
        
        /******************************************************************************************************************
         * Domain associated with problem specification.
         *
         *
         ******************************************************************************************************************/

        struct Associated_Domain
            : seq< s_Domain
                   , pad<Domain_Name, space> > {};
        
        
        /******************************************************************************************************************
         * High level PDDL domain description elements.
         *
         *
         ******************************************************************************************************************/

        struct PDDL_PROBLEM_Element :
            sor<Constants_Description //:constants
                , Associated_Domain//:domain
                , Initial_State
                , Metric
                , Goal
                > {};
        
        struct PDDL_PROBLEM_Element_Noise : seq<Open
                                        , one<':'>, PDDL_PROBLEM_Element
                                        , Close> {};
        
        struct Problem_Description : plus<PDDL_PROBLEM_Element_Noise> {};
        

      struct PDDL_Preamble_Problem : //Metric {};// Associated_Domain {};//Constants_Description {};
            seq< Open
                 , pad<s_Define, space>
                 , Open
                 , s_Problem
                 , pad<Problem_Name, space>
                 , Close
                 , Problem_Description
                 , Close
                 > {};
        
    }
}


#endif

/* From this time, unchained
 * We're all looking at a different picture
 * Thru this new frame of mind
 * A thousand flowers could bloom
 * Move over, and give us some room
 * 
 *  -- Lyrics from Portishead song Glory Box.
 */
