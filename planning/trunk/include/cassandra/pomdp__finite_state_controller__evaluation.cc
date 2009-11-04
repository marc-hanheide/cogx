#include "pomdp__finite_state_controller__evaluation.hh"


using namespace POMDP;
using namespace POMDP::Solving;

FSC::Finite_State_Controller(shared_ptr<Problem_Data> problem_Data,
                             uint number_of_nodes)
    :number_of_nodes(number_of_nodes),
     problem_Data(problem_Data),
     fsc__Index_Management(*this)
{
}

void FSC::zero_initialise()
{
    zero_initialise__node_transition_probabilities();
    zero_initialise__action_execution_probabilities();
}

void FSC::zero_initialise__node_transition_probabilities()
{
    node_transition_probabilities =  decltype(node_transition_probabilities)(number_of_nodes);
    
    for(auto starting_node_index = 0
            ; starting_node_index < number_of_nodes
            ; starting_node_index ++){

        assert(starting_node_index < node_transition_probabilities.size());
        node_transition_probabilities[starting_node_index]
            = vector<vector<vector< double > > >(problem_Data->get__actions_count());
        
        for(auto action_index = 0
                ; action_index < problem_Data->get__actions_count()
                ; action_index++){
            
            assert(action_index < node_transition_probabilities[starting_node_index].size());
            node_transition_probabilities
                [starting_node_index]
                [action_index]
                = vector<vector< double > >(problem_Data->get__observations_count());
            
            for(auto observation_index = 0
                    ; observation_index < problem_Data->get__observations_count()
                    ; observation_index++){
                
                assert(observation_index < node_transition_probabilities[starting_node_index][action_index].size());
                
                node_transition_probabilities
                    [starting_node_index]
                    [action_index]
                    [observation_index]
                    = vector< double >(number_of_nodes);
            
                for(auto successor_node_index = 0
                        ; successor_node_index < number_of_nodes
                        ; successor_node_index ++){

                    assert(successor_node_index
                           < node_transition_probabilities[starting_node_index][action_index][observation_index].size());
                    
                    node_transition_probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        [successor_node_index]
                         = 0.0;
                }
            }
        }
    }
}

void FSC::zero_initialise__action_execution_probabilities()
{
    action_execution_probabilities = decltype(action_execution_probabilities)(number_of_nodes);
    
    for(auto starting_node_index = 0
            ; starting_node_index < number_of_nodes
            ; starting_node_index ++){

        assert(starting_node_index < action_execution_probabilities.size());
        action_execution_probabilities[starting_node_index] = vector< double >(problem_Data->get__actions_count());
        
        
        for(auto action_index = 0
                ; action_index < problem_Data->get__actions_count()
                ; action_index++){

        assert(action_index < action_execution_probabilities[starting_node_index].size());
            
            action_execution_probabilities[starting_node_index]
                [action_index]
                = 0.0;
        }
    }
}

void FSC__Randomizer::randomize__node_transition_probabilities(FSC& fsc)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.number_of_nodes
            ; starting_node_index ++){

        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){
            
            
            for(auto observation_index = 0
                    ; observation_index < fsc.problem_Data->get__observations_count()
                    ; observation_index++){
                
            
                for(auto successor_node_index = 0
                        ; successor_node_index < fsc.number_of_nodes
                        ; successor_node_index ++){
                    
                    assert(starting_node_index < fsc.node_transition_probabilities.size());
                    assert(action_index < fsc.node_transition_probabilities[starting_node_index].size());
                    assert(observation_index < fsc.node_transition_probabilities[starting_node_index][action_index].size());
                    assert(successor_node_index
                           < fsc.node_transition_probabilities[starting_node_index][action_index][observation_index].size());
                    
                    
                    fsc.node_transition_probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        [successor_node_index] = 1.0 / static_cast<double>(fsc.number_of_nodes);
                }
            }
        }
    }
}

void FSC__Randomizer::randomize__action_execution_probabilities(FSC& fsc)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.number_of_nodes
            ; starting_node_index ++){
        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){
            
            assert(starting_node_index < fsc.action_execution_probabilities.size());
            assert(action_index < fsc.action_execution_probabilities[starting_node_index].size());
            
            fsc.action_execution_probabilities[starting_node_index][action_index]
                = 1.0 / static_cast<double>(fsc.problem_Data->get__actions_count());
        }
    }
}

void FSC__Randomizer::operator()(FSC& fsc)
{
    fsc.zero_initialise();
    randomize__node_transition_probabilities(fsc);
    randomize__action_execution_probabilities(fsc);
}

 template<class T>
 bool matrix_inversion__LU_factorisation (const boost::numeric::ublas::matrix<T>& input,
                                          boost::numeric::ublas::matrix<T>& inverse) {
     using boost::numeric::ublas::lu_factorize;
     
     matrix<T> A(input);
     permutation_matrix<std::size_t> pm(A.size1());
     
     bool result = static_cast<bool>(lu_factorize(A,pm));
     if( result != 0 ) return false;
     
     inverse.assign(boost::numeric::ublas::identity_matrix<T>(A.size1()));
     
     lu_substitute(A, pm, inverse);
     
     return true;
 }



int FSC__Index_Management::compute_index__state_node(int state_index, int node_index) const
{
    int state_count = fsc.problem_Data->get__states_count();
    int node_count = fsc.number_of_nodes;

    return state_index + (node_index * state_count);
}

matrix<double> FSC__Evaluator::get_transition_matrix()
{
    matrix<double> state_node__transition__matrix(matrix_dimension, matrix_dimension);

    auto states = fsc.problem_Data->get__states();
    auto actions = fsc.problem_Data->get__actions();
    auto observations = fsc.problem_Data->get__observations();

    auto starting_state_index = 0;
    /* STATES */ for(auto starting_state = states.begin()
                         ; starting_state != states.end()
                         ; starting_state++, starting_state_index++ /* TWO_INCREMENTS */){

        auto successor_state_index = 0;
        /* STATES */ for(auto successor_state = states.begin()
                             ; successor_state != states.end()
                             ; successor_state++, successor_state_index++ /* TWO_INCREMENTS */){

            /* FSC-NODES */ for(auto starting_node = 0
                                   ; starting_node < fsc.number_of_nodes
                                   ; starting_node++){

                /* FSC-NODES */ for(auto successor_node = 0
                                       ; successor_node < fsc.number_of_nodes
                                       ; successor_node++){

                    /* ROW_INDEX */ auto starting_index__row_index
                        = fsc.fsc__Index_Management.compute_index__state_node(
                            starting_state_index, starting_node
                            )
                        ;
                    
                    /* COLUMN_INDEX */auto successor_index__column_index
                        = fsc.fsc__Index_Management.compute_index__state_node(
                            successor_state_index, successor_node);

                    auto transition_probability = 0.0;
                    
                    auto action_index = 0;
                    /* ACTIONS */ for(auto executed_action = actions.begin()
                                          ; executed_action != actions.end()
                                          ; executed_action++, action_index++){

                        auto observation_index = 0;
                        /* OBSERVATIONS */ for(auto observation = observations.begin()
                                                   ; observation != observations.end()
                                                   ; observation++, observation_index++){
                            
                            auto fsc_node_transition_probability
                                = fsc.node_transition_probabilities
                                [starting_node]
                                [action_index]
                                [observation_index]
                                [successor_node];

                            if(0.0 == fsc_node_transition_probability) continue;
                            
                            auto state_transition_probability
                                = fsc.problem_Data->get__transition_probability
                                (*executed_action,
                                 *starting_state,
                                 *successor_state);

                            if(0.0 == state_transition_probability) continue;
                            
                            auto observation_probability
                                = fsc.problem_Data->get__observation_probability//observation_model
                                (*executed_action,
                                 *successor_state,
                                 *observation);
                            
                            if(0.0 == observation_probability) continue;
                            
                            transition_probability +=
                                fsc_node_transition_probability *
                                state_transition_probability *
                                observation_probability;
                            
                            assert(transition_probability < 1.0);
                        }
                    }

                    assert(transition_probability < 1.0);
                    
                    state_node__transition__matrix
                        (starting_index__row_index,
                         successor_index__column_index)
                        = transition_probability;
                    
//                         [starting_index__row_index]
//                         [successor_index__column_index]
//                         = transition_probability;
                }
            }
            
        }
    }
    
    return std::move(state_node__transition__matrix);
}


boost::numeric::ublas::vector<double> FSC__Evaluator::get_reward_vector()
{
    boost::numeric::ublas::vector<double> reward_vector(matrix_dimension);
    
    auto states = fsc.problem_Data->get__states();
    auto actions = fsc.problem_Data->get__actions();
    auto observations = fsc.problem_Data->get__observations();

    
    auto starting_state_index = 0;
    /* STATES */ for(auto starting_state = states.begin()
                         ; starting_state != states.end()
                         ; starting_state++, starting_state_index++ /* TWO_INCREMENTS */){
        
        /* FSC-NODES */ for(auto starting_node = 0
                                ; starting_node < fsc.number_of_nodes
                                ; starting_node++){

            
            /* ROW_INDEX */ auto starting_index__row_index
                = fsc.fsc__Index_Management.compute_index__state_node(
                    starting_state_index, starting_node
                    )
                ;
                    
            auto expected_reward = 0.0;
                    
            
            auto action_index = 0;
            /* ACTIONS */ for(auto executed_action = actions.begin()
                                  ; executed_action != actions.end()
                                  ; executed_action++, action_index++){

                
                auto expected_reward____action__starting_state__successor_state__observation = 0.0;
                
                auto successor_state_index = 0;
                /* STATES */ for(auto successor_state = states.begin()
                                     ; successor_state != states.end()
                                     ; successor_state++, successor_state_index++ /* TWO_INCREMENTS */){

                    auto observation_index = 0;
                    /* OBSERVATIONS */ for(auto observation = observations.begin()
                                               ; observation != observations.end()
                                               ; observation++, observation_index++){

                            
                        auto reward_received
                            = fsc.problem_Data->get__reward
                            (*executed_action,
                             *starting_state,
                             *successor_state,
                             *observation);

                        if(0.0 == reward_received) continue;
                        
                        auto state_transition_probability
                            = fsc.problem_Data->get__transition_probability
                            (*executed_action,
                             *starting_state,
                             *successor_state);
                        
                        if(0.0 == state_transition_probability) continue;
                        
                        auto observation_probability
                            = fsc.problem_Data->get__observation_probability//observation_model
                            (*executed_action,
                             *successor_state,
                             *observation);
                            
                        if(0.0 == observation_probability) continue;
                        
                        
                        expected_reward____action__starting_state__successor_state__observation
                            += reward_received *
                            state_transition_probability *
                            observation_probability;
                        
                    }
                }
                
                auto probability_of_executed_action
                    = fsc.action_execution_probabilities
                    [starting_node]
                    [action_index];
                

                expected_reward
                    += probability_of_executed_action *
                    expected_reward____action__starting_state__successor_state__observation;
            }

            reward_vector(starting_index__row_index) = expected_reward;
        }
    }

    return std::move(reward_vector);
}

double FSC__Evaluator::operator()(const Belief_State& belief_State)
{
    
}

void FSC__Evaluator::operator()()
{
    /*MEMBER*/ state_count = fsc.problem_Data->get__states_count();
    /*MEMBER*/ node_count = fsc.number_of_nodes;
    /*MEMBER*/ matrix_dimension = state_count * node_count;

    /* A */ matrix<double> state_node__transition__matrix
        = get_transition_matrix();

    /* R */ boost::numeric::ublas::vector<double> instantanious_reward_vector
        = get_reward_vector();
    

    /*MEMBER*/ value_vector
        = boost::numeric::ublas::vector<double>(zero_vector<double>(matrix_dimension));

    
    auto discount_factor = fsc.problem_Data->get__discount_factor();
    
    /* SOLVING SYSTEM -- INVERTING */
    matrix<double> INVERSE__transition__matrix(matrix_dimension, matrix_dimension);
    
    /* INVERTING (I - A) */ matrix<double> transition__matrix(matrix_dimension, matrix_dimension);
    /* INVERTING (I - A) */ transition__matrix.assign(identity_matrix<double>(matrix_dimension));
    /* INVERTING (I - A) */ transition__matrix -= (discount_factor * state_node__transition__matrix);

    /* INVERTING  */ if(!matrix_inversion__LU_factorisation(transition__matrix, INVERSE__transition__matrix)){
    /* INVERTING  */     UNRECOVERABLE_ERROR("Unable to take inverse of state-transition matrix.");
    /* INVERTING  */ }
    
    /* INVERSE (I - A) */ //invert(tmp);

    /* \MEMBER{value_vector} ---- (I - A)^{-1} * R */
    value_vector.assign(prod(INVERSE__transition__matrix, instantanious_reward_vector));


    
}

FSC__Evaluator::Finite_State_Controller__Evaluator(const FSC& fsc)
    :fsc(fsc)
{   
}

std::ostream& POMDP::Solving::operator<<(std::ostream& o, const FSC__Evaluator& fsc__Evaluator)
{
    const FSC& fsc = fsc__Evaluator.fsc;
    auto state_count = fsc.problem_Data->get__states_count();
    auto node_count = fsc.number_of_nodes;
    
    for(uint state_index = 0; state_index < state_count; state_index++){
        for(uint node_index = 0; node_index < node_count; node_index++){
            /* ROW_INDEX */ auto row_index
                = fsc.fsc__Index_Management.compute_index__state_node(
                    state_index, node_index
                    )
                ;

            decltype(fsc.problem_Data->get__states()) states
                = fsc.problem_Data->get__states();

            assert(state_index < states.size());
            auto state_string = states[state_index];

            QUERY_WARNING(row_index >= fsc__Evaluator.value_vector.size(),
                          "Trying to print value information before having computed it.");
            
            assert(row_index < fsc__Evaluator.value_vector.size());
            
            o<<"("<<state_string<<" x "<<node_index<<") == "
             <<fsc__Evaluator.value_vector[row_index]<<std::endl;
        }
    }
    
    return o<<std::endl;
}
