#ifndef DTP_PDDL_PARSING_ACTIONS_HH
#define DTP_PDDL_PARSING_ACTIONS_HH

#include <pegtl.hh>

#include "dtp_pddl_parsing_data.hh"

namespace Planning
{
    namespace Parsing
    {
        using namespace pegtl;

//         typedef Data stack_type;

        struct Function__Action : action_base< Function__Action >
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__function_component();
                VERBOSER(1, str);
            }
        };
        
        struct Make_Action__Action : action_base< Make_Action__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__action_description();
                VERBOSER(1, str);
            }
        };
        
        
        struct Make_Perception__Action : action_base< Make_Perception__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__perception_description();
                VERBOSER(1, str);
            }
        };
        
        struct Variable__Action : action_base< Variable__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__variable(str);
                VERBOSER(1, str);
            }
        };
        
        struct Type__Action : action_base< Type__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__type(str);
                VERBOSER(1, str);
            }
        };
        
        struct Constant__Action : action_base< Constant__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__constant(str);
                VERBOSER(1, str);
            }
        };
        
        
        struct Domain_Constants__Action : action_base< Domain_Constants__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.commit__constants();
                VERBOSER(1, str);
            }
        };
        
        
        struct Requirement__Action : action_base< Requirement__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__requirement(str);
                VERBOSER(1, str);
            }
        };
        
        struct Types__Action : action_base< Types__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.commit__types();
                VERBOSER(1, str);
            }
        };
        
        struct Build_Argument__Action : action_base< Build_Argument__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__arguments_description_component();
                VERBOSER(1, str);//"combined some argument variables with their types.");
            }
        };

        struct Action_Name__Action : action_base< Action_Name__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.report__action_name(str);
                VERBOSER(1, str);
            }
        };

        struct Perception_Name__Action : action_base< Perception_Name__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.report__perception_name(str);
                VERBOSER(1, str);
            }
        };

        struct Domain_Name__Action : action_base<Domain_Name__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.report__domain_name(str);
                VERBOSER(1, str);
            }
        };
        
        struct Predicate_Name__Action : action_base< Predicate_Name__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.report__predicate_name(str);
                VERBOSER(1, str);
            }
        };
        
        struct Action_Signature__Action : action_base< Action_Signature__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__action_signature();
                VERBOSER(1, str);
            }
        };

        struct Perception_Signature__Action : action_base< Perception_Signature__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__perception_signature();
                VERBOSER(1, str);
            }
        };

        struct Function_Name__Action : action_base< Function_Name__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.report__function_name(str);
                VERBOSER(1, str);
            }
        };
        
        struct Function_Type_Number__Action : action_base< Function_Type_Number__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.set_function_range__number();
                VERBOSER(1, str);
            }
        };

        struct Function_Type_Int__Action : action_base< Function_Type_Int__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.set_function_range__int();
                VERBOSER(1, str);
            }
        };

        struct Function_Type_Double__Action : action_base< Function_Type_Double__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.set_function_range__double();
                VERBOSER(1, str);
            }
        };

        struct Predicate_Description__Action : action_base< Predicate_Description__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__predicate_description_component();
                VERBOSER(1, str);
            }
        };

        struct Perception_Action__Action : action_base< Perception_Action__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.reinterpret__predicate_as_perception_action_precondition();
                VERBOSER(1, str);
            }
        };

        struct Function_Description__Action : action_base< Function_Description__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.combine__function_description_component();
                VERBOSER(1, str);
            }
        };

        struct Domain_Predicates__Action : action_base< Domain_Predicates__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__domain_predicates();
                VERBOSER(1, str);
            }
        };

        struct Domain_Functions__Action : action_base< Domain_Functions__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__domain_functions();
                VERBOSER(1, str);
            }
        };

        struct Not_Precondition__Action : action_base< Not_Precondition__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__negative_precondition();
                VERBOSER(1, str);
            }
        };
        
        struct Negative_Effect__Action : action_base< Negative_Effect__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__negative_effect();
                VERBOSER(1, str);
            }
        };
        
        struct Predicate_Precondition__Action : action_base< Predicate_Precondition__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__predicate_precondition();
                VERBOSER(1, str);
            }
        };
        
        struct Number__Action : action_base< Number__Action>
        {
            
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s.add__number(str);
                VERBOSER(1, str);
            }
        };
    }
}
  

#endif
