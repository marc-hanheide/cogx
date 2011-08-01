

/* Copyright (C) 2009 Charles Gretton (charles.gretton@gmail.com)
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


#include "cassandra_POMDP__parser.hh"

using namespace POMDP::Parsing;


// const string& Problem_Data::get__state(int index) const
// {
//     return states[index];
// }

// const string& Problem_Data::get__action(int index) const
// {
//     return actions[index];
// }

    
// const string& Problem_Data::get__observation(int index) const
// {
//     return observations[index];
// }


uint Problem_Data::get__states_count() const
{
    return states.size();
}

uint Problem_Data::get__actions_count() const
{
    return actions.size();
}

uint Problem_Data::get__observations_count() const
{
    return observations.size();
}

const decltype(Problem_Data::states)& Problem_Data::get__states() const
{
    return states;
}

const decltype(Problem_Data::actions)& Problem_Data::get__actions() const
{
    return actions;
}

const decltype(Problem_Data::observations)& Problem_Data::get__observations() const
{
    return observations;
}


#define IMPLEMENT__GET_MEMBER_FUNCTION__WITH_ARGUMENT(FUNCTION_NAME, INDEX_TYPE, CLASS, RETURN_TYPE, DATA) \
    RETURN_TYPE CLASS::FUNCTION_NAME(INDEX_TYPE index) const            \
    {                                                                   \
        assert(DATA.valid(index));                                      \
        return DATA.get(index);                                         \
    }                                                                   \
    

IMPLEMENT__GET_MEMBER_FUNCTION__WITH_ARGUMENT
(get__state,
 int,
 Problem_Data,
 const std::string&,
 states)
    
IMPLEMENT__GET_MEMBER_FUNCTION__WITH_ARGUMENT
(get__action,
 int,
 Problem_Data,
 const std::string&,
 actions)
    
IMPLEMENT__GET_MEMBER_FUNCTION__WITH_ARGUMENT
(get__observation,
 int,
 Problem_Data,
 const std::string&,
 observations)

double Problem_Data::get__reward(int executed_action__index,
                                 int starting_state__index,
                                 int successor_state__index,
                                 int observation__index) const
{
    assert(executed_action__index < actions.size());
    assert(starting_state__index < states.size());
    assert(successor_state__index < states.size());
    assert(observation__index < observations.size());
    
    return get__reward(actions[executed_action__index],
                       states[starting_state__index],
                       states[successor_state__index],
                       observations[observation__index]);
}

            
double Problem_Data::get__transition_probability(int executed_action__index,
                                                 int starting_state__index,
                                                 int successor_state__index) const
{
    assert(executed_action__index < actions.size());
    assert(starting_state__index < states.size());
    assert(successor_state__index < states.size());

    return get__transition_probability(actions[executed_action__index],
                                       states[starting_state__index],
                                       states[successor_state__index]);
    
}

            
double Problem_Data::get__observation_probability(int executed_action__index,
                                                  int successor_state__index,
                                                  int observation__index) const
{
    assert(executed_action__index < actions.size());
    assert(successor_state__index < states.size());
    assert(observation__index < observations.size());

    return get__observation_probability(actions[executed_action__index],
                                        states[successor_state__index],
                                        observations[observation__index]);
}




double Problem_Data::get__reward(const std::string& executed_action__name,
                                 const std::string& starting_state__name,
                                 const std::string& successor_state__name,
                                 const std::string& observation__name) const
{
    auto reward_model__action__level
        = reward_model.find(executed_action__name);

    assert(reward_model__action__level != reward_model.end());

    auto reward_model__starting_state__level
        = reward_model__action__level->second.find(starting_state__name);

    assert(reward_model__starting_state__level != reward_model__action__level->second.end());

    
    QUERY_UNRECOVERABLE_ERROR(reward_model__starting_state__level->second.size() != states.size(),
                              "Insufficient reward information for :: "
                              <<reward_model__action__level->first<<" "
                              <<reward_model__starting_state__level->first<<" "
                              <<" that contains "<<reward_model__starting_state__level->second.size()<<" entries "
                              <<"whereas there are "<<states.size()<<" states."
                              <<std::endl);
    
    auto reward_model__successor_state__level
        = reward_model__starting_state__level->second.find(successor_state__name);
    
    QUERY_UNRECOVERABLE_ERROR(reward_model__successor_state__level == reward_model__starting_state__level->second.end(),
                              "Successor state "<<successor_state__name
                              <<" is not associated with any reward information."<<std::endl);

    
    QUERY_UNRECOVERABLE_ERROR(reward_model__successor_state__level->second.size() != observations.size(),
                              "Insufficient reward information for :: "
                              <<reward_model__action__level->first<<" "
                              <<reward_model__starting_state__level->first<<" "
                              <<reward_model__successor_state__level->first<<" "
                              <<" that contains "<<reward_model__successor_state__level->second.size()<<" entries "
                              <<"whereas there are "<<observations.size()<<" observations."
                              <<std::endl);
    
    auto reward_model__observation__level
        = reward_model__successor_state__level->second.find(observation__name);
    
    QUERY_UNRECOVERABLE_ERROR(reward_model__observation__level == reward_model__successor_state__level->second.end(),
                              "Observation "<<observation__name
                              <<" is not associated with any reward information."<<std::endl);
    
    assert(reward_model__observation__level != reward_model__successor_state__level->second.end());

    return reward_model__observation__level->second;
}



double Problem_Data::get__transition_probability(const std::string& executed_action__name,
                                                 const std::string& starting_state__name,
                                                 const std::string& successor_state__name) const
{
    
    QUERY_UNRECOVERABLE_ERROR(transition_model.size() != actions.size(),
                              "We have "<<actions.size()
                              <<" actions, however only "<<transition_model.size()
                              <<" transition entries"<<std::endl);
    
    auto transition_model__action__level
        = transition_model.find(executed_action__name);

    QUERY_UNRECOVERABLE_ERROR(transition_model__action__level == transition_model.end(),
                              "Action "<<executed_action__name
                              <<" is not associated with any transition information."<<std::endl);
    
    assert(transition_model__action__level != transition_model.end());

    auto transition_model__starting_state__level =
        transition_model__action__level->second.find(starting_state__name);
    
    QUERY_UNRECOVERABLE_ERROR(transition_model__starting_state__level == transition_model__action__level->second.end(),
                              "Starting-State "<<starting_state__name
                              <<" is not associated with any transition information of action :: "<<executed_action__name<<std::endl);
    assert(transition_model__starting_state__level != transition_model__action__level->second.end());

    auto transition_model__successor_state__level =
        transition_model__starting_state__level->second.find(successor_state__name);

    QUERY_UNRECOVERABLE_ERROR(transition_model__successor_state__level == transition_model__starting_state__level->second.end(),
                              "Successor-State "<<starting_state__name
                              <<" is not associated with any transition information of action :: "
                              <<executed_action__name
                              <<" for starting-state :: "<<starting_state__name<<std::endl);
    
    assert(transition_model__successor_state__level != transition_model__starting_state__level->second.end());
    

    VERBOSER(1, "Transition probability is :: "<<transition_model__successor_state__level->second<<std::endl);
    
    return transition_model__successor_state__level->second;
}

            
double Problem_Data::get__observation_probability(const std::string& executed_action__name,
                                                  const std::string& successor_state__name,
                                                  const std::string& observation__name) const
{
    auto observation_model__action__level
        = observation_model.find(executed_action__name);

    assert(observation_model__action__level != observation_model.end());

    auto observation_model__successor_state__level
        = observation_model__action__level->second.find(successor_state__name);

    assert(observation_model__successor_state__level != observation_model__action__level->second.end());
    
    auto observation_model__observation__level
        = observation_model__successor_state__level->second.find(observation__name);

    assert(observation_model__observation__level != observation_model__successor_state__level->second.end());
    
    return observation_model__observation__level->second;
}

double Problem_Data::get__discount_factor() const
{
    return discount_factor;
}
