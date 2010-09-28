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


#ifndef POLICY_ITERATION_OVER_INFORMATION_STATE_SPACE_HH
#define POLICY_ITERATION_OVER_INFORMATION_STATE_SPACE_HH

#include "partially_observable_markov_decision_process_state.hh"

#include <boost/numeric/ublas/blas.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

namespace Planning
{
    class Policy_Iteration
    {
    public:
        Policy_Iteration(Set_Of_POMDP_State_Pointers&,
                         double sink_state_penalty, 
                         double discount_factor = 0.98);

        void operator()();
        
    private:
        void press_greedy_policy();
        boost::numeric::ublas::vector<double> get_reward_vector();
        boost::numeric::ublas::matrix<double> get_transition_matrix2();
        boost::numeric::ublas::matrix<double> get_transition_matrix();
        
        Set_Of_POMDP_State_Pointers& states;

        uint dimension;

        double discount_factor;
        double sink_state_penalty;
        
        boost::numeric::ublas::vector<double> instantanious_reward_vector;
        boost::numeric::ublas::vector<double> value_vector;
        boost::numeric::ublas::matrix<double> state_transition_matrix;
    };
        
}




#endif

/*  ... Before the war <<WW2>> he had done it to Mauriac. The great
 * Catholic writer had just published {\em La Fin De La Nuit}. It was a
 * classic. It was praised to the skies by the critics. Well received by
 * readers. In hindsight, it is one of Mauriac's really fine novels. Then
 * along came Sartre, who, in an article in the {\em Nouvelle Revue
 * Francais}, explained that the whole art of the older writer rested on
 * the postulate that the writer `plays the same role towards his
 * creatures as God does towards his`, and that he, Sartre, was in a
 * position to assert, from the heights of the legitimacy that the
 * publication of {\em Nausea} alone, a year earlier, had just granted
 * him, that what we had here was an unforgivable `technical error`
 * compounded by a philosophical blunder; the novelist `is not God`; he
 * `does not have the right to pronounce his absolute judgements` or to
 * manipulate his characters as puppets in the way Mauriac did; he does
 * not have the right to enter into their reasons or their unreasons,
 * their paltry contradictions so obviously strung together by the
 * omniscient narrator's designs on them, and so here is the verdict he
 * felt obliged to deliver: {\em La Fin De La Nuit} `is not a novel`, for
 * `God is not an artist and neither is Monsieur Mauriac`. The victim
 * swallows the insult. He didn't respond, he took it on the chin, and,
 * playing a close game, didn't disdain, as the years went by -- for
 * example, at the time of Sartre's preface to {\em Aden Arabie} -- to
 * pay homage to his adversary's talent. But he waited thirty years
 * before risking publishing a new novel. Sartre, he said, tried `from
 * the outset, twenty years ago, [...] to throttle me`. And, on another
 * occasion, in answer to a journalist from {\em France-Soir} who asked
 * him what was the reason behind such a long silence, he replies: {\em
 * La Fin De La Nuit} had been `panned by Sartre`, who was `not only a
 * very young author, but at the same time the glory of his generation`;
 * and without going so far as to say that this `attack` demoralised him,
 * he admitted that it had `made me think`. Sartre was to regret the way
 * he had panned the novel. He later agreed, in 1960, that `all methods
 * are a form of rigging`, and that the celebrated `American methods`
 * that he had claimed, at the time, to be upholding in opposition to
 * Mauriac's literary interventionism were scarcely less artificial. But
 * that's how it was, in the final analysis. It was a period when a
 * literary panning, published in a review, could give you something to
 * think about for thirty years. A period, in 1939, the middle of the
 * twentieth century, when Sartre, still so very young, already had this
 * power to destroy a novelist...
 * 
 * -- Bernard-Henri L\'evy, Sartre: The Philosopher of the Twentieth
 *    Century, Polity, 2003.
 * 
 */
