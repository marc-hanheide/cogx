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


#ifndef POMDP__FINITE_STATE_CONTROLLER_HH
#define POMDP__FINITE_STATE_CONTROLLER_HH

#include "utilities.hh"
#include "cassandra_POMDP__parser.hh"


namespace POMDP
{
    using std::tr1::shared_ptr;
//     using std::vector;
    using POMDP::Parsing::Problem_Data;

    
    class Finite_State_Controller;
    
    class Finite_State_Controller__Index_Management
    {
    public:
        Finite_State_Controller__Index_Management(const Finite_State_Controller& fsc):fsc(fsc){};

        /* Gives the numeric index associated with a state (at
         * \argument{state_index}) and a node (at
         * \argument{node_index}). */
        int compute_index__state_node(int state_index, int node_index) const;
    private:
        const Finite_State_Controller& fsc;
    };
        
    typedef Finite_State_Controller__Index_Management FSC__Index_Management;

    class Finite_State_Controller;
    
    std::ostream& operator<<(std::ostream&, const Finite_State_Controller&);
    
    class Finite_State_Controller
    {
    public:
        friend std::ostream& operator<<(std::ostream&, const Finite_State_Controller&);
        
        /* (see \member{number_of_nodes})*/
        int get__nodes_count() const;
        /* (see \member{number_of_nodes})*/
        int increment__nodes_count(int amount);
        /* (see \member{number_of_nodes})*/
        int decrement__nodes_count(int amount);
        
        /*\argument{problem_Data} is the
         * POMDP. \argument{number_of_nodes} gives the number of
         * controller nodes.*/
        Finite_State_Controller(shared_ptr<Problem_Data> problem_Data, uint number_of_nodes = 3);

        /* Add space for entries in
         * \member{node_transition_probabilities},
         * \member{action_execution_probabilities}.*/
        void zero_initialise();
        void zero_initialise__node_transition_probabilities();
        void zero_initialise__action_execution_probabilities();
        
        void randomize();

        
        typedef std::vector<std::vector<std::vector<std::vector< double > > > > Node_Transition_Probabilities;
        typedef std::vector<std::vector<std::vector< double > > > Node_Transition_Probabilities__Starting_Nodes_Layer;
        typedef std::vector<std::vector< double > > Node_Transition_Probabilities__Actions_Layer;
        typedef std::vector< double > Node_Transition_Probabilities__Observations_Layer;
        typedef Node_Transition_Probabilities NTP;
        typedef Node_Transition_Probabilities__Starting_Nodes_Layer NTP__Starting_Nodes_Layer;
        typedef Node_Transition_Probabilities__Actions_Layer NTP__Actions_Layer;
        typedef Node_Transition_Probabilities__Observations_Layer NTP__Observations_Layer;
        
        /* starting-node -> action -> observation -> successor-node -> double*/
        Node_Transition_Probabilities  node_Transition_Probabilities;        

        typedef std::vector<std::vector< double > > Action_Execution_Probabilities;
        typedef std::vector< double > Action_Execution_Probabilities__Starting_Nodes_Layer;
        typedef Action_Execution_Probabilities AEP;
        typedef Action_Execution_Probabilities__Starting_Nodes_Layer AEP__Starting_Nodes_Layer;
        
        /* starting-node -> action -> double*/
        Action_Execution_Probabilities action_Execution_Probabilities;
        
        
        /*Assigned problem*/
        shared_ptr<Problem_Data> problem_Data;



        /* Set a single entry in
         * \member{action_execution_probabilities}. In particular,
         * \argument{psi_entry} gives the probability of executing
         * \argument{action_index} given we are in controller node
         * (i.e., fsc node) \argument{node_index}.*/
        void set__action_execution_probability(int node_index,
                                               int action_index,
                                               double action_choice_probability);
        
        /* Set a single entry in
         * \member{node_transition_probabilities}. In particular,
         * \argument{transition_probability} gives the probability of
         * a transition from fsc node \argument{starting_node_index}
         * to fsc node \argument{successor_node_index} given we
         * execute action \argument{action_index} at node
         * \argument{starting_node_index} and receive observation
         * \argument{observation_index}.*/
        void set__node_transition_probability(int action_index,
                                              int observation_index,
                                              int starting_node_index,
                                              int successor_node_index,
                                              double transition_probability);

        
        bool sanity__node_transition_probabilities() const;
        bool sanity__action_execution_probabilities() const;
        
        Finite_State_Controller__Index_Management fsc__Index_Management;

    private:
        /*Number of controller nodes*/
        int number_of_nodes;
    };

    typedef Finite_State_Controller FSC;

    
//     using boost::test_toolbox::close_at_tolerance;
    class Finite_State_Controller__Randomizer
    {
    public:
        Finite_State_Controller__Randomizer();
        
        void operator()(FSC&);

    private:
        void randomize__node_transition_probabilities(FSC&);
        void randomize__action_execution_probabilities(FSC&);
    };

    typedef Finite_State_Controller__Randomizer FSC__Randomizer;
        
}


#endif
