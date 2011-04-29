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

#ifndef ACTION__STATE_TRANSFORMATION_HH
#define ACTION__STATE_TRANSFORMATION_HH

#include "planning_formula.hh"
#include "planning_types_enum.hh"
#include "state_basics.hh"
#include "planning_formula.hh"
#include "state_formula.hh"

namespace Planning
{
    
    /* We do not explicitly store "transformation" preconditions
     * (i.e., a PDDL action preconditions) in the transformation, but
     * rather store those separately as CNF formulae. When a formula
     * is true at some given state, then \class{Satisfaction_Listener}
     * that are registered with that formula get notified (see
     * \module{state_formula.hh}). */
    class State_Transformation :
        public State_Formula::
        _Satisfaction_Listener<enum_types::state_transformation
                               , Formula::Action_Proposition
                               , State_Formula::Conjunctive_Normal_Form_Formula__Pointer/*TODO deal with empty conjunct.*/
                               , State_Formula::List__Literals /* effects */
                               , bool /* compulsory -- whole action */
                               , bool /* lookup probability */
                               , double /* Probability that this transformation is applied. */
                               , uint /* Probability lookup index (ignore if "lookup probability" is false). */>
    {PRINTING;
        
    public:
        /* How is this action identified in the other modules of this
         * system? This will be a PDDL-based identifier if the
         * transformation corresponds to a ground PDDL operator.*/
        const Formula::Action_Proposition& get__identifier() const;

        /* What conditions on a state must be satisfied in order for
         * this transformation to be applicable? That condition is
         * expressed as a conjunctive normal form propositional
         * formula (see \module{state_formula}). */
        const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& get__precondition() const;
        /* What are the add and delete effects of this transformation? */
        const State_Formula::List__Literals& get__effects() const;

        /* A compulsory transformation, is one that must be applied to
         * a state. That is, the agent cannot choose to execute it, or
         * choose not to execute it. */
        bool get__compulsory() const;

        /* Should the probability of application be read from a state?*/
        bool get__lookup_probability() const;

        /*What is the probability of successful application of this transformation?*/
        double get__probability() const;

        /* If the probability of a successful transformation is to be
         * read from a state, then this member reads that
         * information. */
        double get__probability(const State&) const;
        
//         virtual ~State_Transformation(){};

//         typedef std::vector<State*> Result_Type;
        
        /* Should be repeatedly executed until the result is NULL, or
         * no pending actions are required. If the \return{bool} is
         * true, then this method should be called again on the same
         * input state, etc until the \return{bool} is false.
         *
         * Sometimes the result has a different address from the
         * input. Here, the transformation has been generative.
         *
         * \argument{predecessor} is the state that transitions to
         * \argument{successor}. The latter is the state being
         * generated. \argument{SetOfStatePointers} is the set of
         * problem states thus far discovered.*/
         State* operator()(State* predecessor) const;
        
        void report__newly_satisfied(State&) const;
        void report__newly_unsatisfied(State&) const;

        /* Changes the executability status in \argument{State} of the
         * transformation object.*/
        void flip(State&) const;
        
        void forced_wake(State&) const;
        
        bool is_satisfied(const State&) const;
            
        uint get__level_of_satisfaction(State&) const;


        uint get__number_of_satisfied_conditions(State& state) const;

    private:
        void set__level_of_satisfaction(uint, State&) const;
        void increment__level_of_satisfaction(State&) const;
        void decrement__level_of_satisfaction(State&) const;
        void set__satisfied(State&) const;
        void set__unsatisfied(State&) const;
        
        static Are_Doubles_Close are_Doubles_Close;//(1e-9);
    };
    
}

#endif


/* Given the existence as uttered forth in the public works of Puncher
 * and Wattman of a personal God quaquaquaqua with white beard
 * quaquaquaqua outside time without extension who from the heights of
 * divine apathia divine athambia divine aphasia loves us dearly with
 * some exceptions for reasons unknown but time will tell and suffers
 * like the divine Miranda with those who for reasons unknown but time
 * will tell are plunged in torment plunged in fire whose fire flames
 * if that continues and who can doubt it will fire the firmament that
 * is to say blast hell to heaven so blue still and calm so calm with a
 * calm which even though intermittent is better than nothing but not
 * so fast and considering what is more that as a result of the labors
 * left unfinished crowned by the Acacacacademy of Anthropopopometry of
 * Essy-in-Possy of Testew and Cunard it is established beyond all
 * doubt all other doubt than that which clings to the labors of men
 * that as a result of the labors unfinished of Testew and Cunard it is
 * established as hereinafter but not so fast for reasons unknown that
 * as a result of the public works of Puncher and Wattman it is
 * established beyond all doubt that in view of the labors of Fartov
 * and Belcher left unfinished for reasons unknown of Testew and Cunard
 * left unfinished it is established what many deny that man in Possy
 * of Testew and Cunard that man in Essy that man in short that man in
 * brief in spite of the strides of alimentation and defecation wastes
 * and pines wastes and pines and concurrently simultaneously what is
 * more for reasons unknown in spite of the strides of physical culture
 * the practice of sports such as tennis football running cycling
 * swimming flying floating riding gliding conating camogie skating
 * tennis of all kinds dying flying sports of all sorts autumn summer
 * winter winter tennis of all kinds hockey of all sorts penicilline
 * and succedanea in a word I resume flying gliding golf over nine and
 * eighteen holes tennis of all sorts in a word for reasons unknown in
 * Feckham Peckham Fulham Clapham namely concurrently simultaneously
 * what is more for reasons unknown but time will tell fades away I
 * resume Fullham Clapham in a word the dead loss per head since the
 * death of Bishop Berkeley being to the tune of one inch four ounce
 * per head approximately by and large more or less to the nearest
 * decimal good measure round figures stark naked in the stockinged
 * feet in Connemara in a word for reasons unknown no matter what
 * matter the facts are there and considering what is more much more
 * grave that in the light of the labors lost of Steinweg and Peterman
 * it appears what is more much more grave that in the light the light
 * the light of the labors lost of Steinweg and Peterman that in the
 * plains in the mountains by the seas by the rivers running water
 * running fire the air is the same and then the earth namely the air
 * and then the earth in the great cold the great dark the air and the
 * earth abode of stones in the great cold alas alas in the year of
 * their Lord six hundred and something the air the earth the sea the
 * earth abode of stones in the great deeps the great cold on sea on
 * land and in the air I resume for reasons unknown in spite of the
 * tennis the facts are there but time will tell I resume alas alas on
 * on in short in fine on on abode of stones who can doubt it I resume
 * but not so fast I resume the skull fading fading fading and
 * concurrently simultaneously what is more for reasons unknown in
 * spite of the tennis on on the beard the flames the tears the stones
 * so blue so calm alas alas on on the skull the skull the skull the
 * skull in Connemara in spite of the tennis the labors abandoned left
 * unfinished graver still abode of stones in a word I resume alas alas
 * abandoned unfinished the skull the skull in Connemara in spite of
 * the tennis the skull alas the stones Cunard (mêlée, final
 * vociferations) tennis . . . the stones . . . so calm . . . Cunard
 * . . . unfinished . . .
 * 
 * \end{quote}
 *
 * Lucky speaks in Samuel Beckett's Waiting for Godot, pp. 44-7, ISBN:
 * 0-8021-1821-6 / ISBN-13: 978-0-8021-1821-9, 2006.
 *
 * (see \url{http://www.groveatlantic.com/grove/bin/wc.dll?groveproc~genauth~56~5215~EXCERPT})
 */


