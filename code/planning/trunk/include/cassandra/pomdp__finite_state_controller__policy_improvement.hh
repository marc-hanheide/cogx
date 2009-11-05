#ifndef POMDP__FINITE_STATE_CONTROLLER__POLICY_IMPROVEMENT_HH
#define POMDP__FINITE_STATE_CONTROLLER__POLICY_IMPROVEMENT_HH

#include "utilities.hh"
#include "pomdp__finite_state_controller.hh"


namespace POMDP
{
    namespace Solving
    {
        class Finite_State_Controller__Node_Improvement
        {
        public:
            Finite_State_Controller__Node_Improvement(FSC& fsc);
            
            void operator()(int node_index);
        private:
            /*Controller*/
            FSC& fsc;
        };
        
        typedef Finite_State_Controller__Node_Improvement FSC__Node_Improvement;
        
        class Finite_State_Controller__Improvement
        {   
        public:
            /* For each node of \argument{finite_State_Controller},
             * improve the controller at that node according to
             * \class{Finite_State_Controller__Node_Improvement}*/
            bool operator()(){};
        private:
        };
        
        typedef Finite_State_Controller__Randomizer FSC__Improvement;
    }
}

        

#endif
