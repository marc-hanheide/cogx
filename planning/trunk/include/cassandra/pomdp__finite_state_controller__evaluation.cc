#include "pomdp__finite_state_controller__evaluation.hh"


using namespace POMDP;
using namespace POMDP::Solving;

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

double FSC__Evaluator::operator()(const Belief_State& belief_State, int node_index)
{
    /*MEMBER*/ state_count = fsc.problem_Data->get__states_count();

    assert(belief_State.size() == state_count);

    double result = 0.0;
    
    for(auto state_index = 0
            ; state_index < state_count
            ; state_index++){
        
         /* ROW_INDEX */ auto row_index
                = fsc.fsc__Index_Management.compute_index__state_node(
                    state_index, node_index
                    )
                ;

         assert(state_index < belief_State.size());
         result += belief_State[state_index] * value_vector[row_index];
         
    }

    return result;
}

double FSC__Evaluator::operator()(const Belief_State& belief_State)
{

    double max_value = 0.0;
    double max_node = 0;

    for(auto node_index = 0; node_index < fsc.number_of_nodes; node_index++){
        auto node_value = (*this)(belief_State);

        if(node_value > max_value){
            max_value = node_value;
            max_node = node_index;
        }
    }

    optimal_starting_node = max_node;
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
