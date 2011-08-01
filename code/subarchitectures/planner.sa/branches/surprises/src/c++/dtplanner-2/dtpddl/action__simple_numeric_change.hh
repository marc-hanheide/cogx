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
#ifndef ACTION__SIMPLE_NUMERIC_CHANGE_HH
#define ACTION__SIMPLE_NUMERIC_CHANGE_HH


#include "planning_formula.hh"
#include "state_formula.hh"
#include "state_basics.hh"
#include "planning_types_enum.hh"


namespace Planning
{
    
    /* Supposing the \argument{formula} is an action description, we
     * count how many parts of that action description could cause
     * reward to be allocated at a successor state should that action
     * be executed. The result is cumulatively added to
     * \argument{state} via
     * \member{increment__obtainable_rewards_count}.
     *
     * \argument{reward_index} is the index of the reward function
     * evaluation at MDP planning states (see \module{markov_decision_process_state}).
     *
     * \argument{state} is the planning state at which we are counting
     * available rewards at formula. Indeed, the number of available
     * rewards from \argument{formula} are accumulated in
     * \argument{state}.
     *
     * The rewards that are made possible at this state by the action
     * formula \argument{formula} are accumulated at \argument{state}
     * \member{obtainable_rewards_value}.
     */
    void count_reward_assignments_at_state(State_Formula::Satisfaction_Listener__Pointer formula,
                                           State& state,
                                           uint reward_index = 0);
}


namespace Planning
{

    
    template<typename Range_Type, int type_id>
    class Simple_Numeric_Transformation :
        public State_Formula::
        _Satisfaction_Listener<type_id
                               , Formula::Action_Proposition
                               , ID_TYPE /* Index to change */
                               , Range_Type
                               , int/*enum*/>
    {PRINTING;
    public:
        
        typedef  State_Formula::
        _Satisfaction_Listener<type_id
                               , Formula::Action_Proposition
                               , ID_TYPE /* Index to change */
                               , Range_Type
                               , int/*enum*/> Parent;

        Simple_Numeric_Transformation()
            :call_count(0)
        {
        }
        
        
        const Formula::Action_Proposition& get__identifier() const;
        ID_TYPE get__change_index() const;// {return std::tr1::get<1>(Parent::contents());};
        Range_Type get__modification_value() const;
        int get__modification_type() const;
        
        void forced_wake(State&) const;
        
        State* operator()(State*) const;
        
        
        void report__newly_satisfied(State&) const{assert(0);};
        void report__newly_unsatisfied(State&) const{assert(0);};
    protected:
        mutable uint call_count;
    };
    
    class Simple_Int_Transformation
    : public Simple_Numeric_Transformation<
        int
        , enum_types::simple_int_transformation>
    {
    public:
        
        void set__statically_false(State& state) const;
    };
    
    class Simple_Double_Transformation
    : public Simple_Numeric_Transformation<
        double
        , enum_types::simple_double_transformation> {};

}

#endif

/* On June 12, 1979 the Gossamer Albatross completed a successful
 * crossing of the English Channel to win the second Kremer prize. It
 * was an aircraft that was powered by the cyclist Bryan Allen. */
