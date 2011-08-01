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
 * CogX ::
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

/* "Actions" in the sense of this module are PEG(TL) ---i.e., parsing
 * expression grammar--- actions. Hence all the functionality we
 * provide here occurs in the namespace Planning::Parsing
 *
 */

#ifndef DTP_PDDL_PARSING_ACTIONS_HH
#define DTP_PDDL_PARSING_ACTIONS_HH


#define DECLARATION__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME) \
    struct ACTION_NAME : action_base< ACTION_NAME >                     \
    {                                                                   \
        template<typename stack_type>                                   \
            static void apply(const std::string& str, stack_type& s)    \
        {                                                               \
            UNRECOVERABLE_ERROR("UNIMPLEMENTED");                       \
        }                                                               \
    }                                                                   \
        
#define DECLARATION__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT) \
    struct ACTION_NAME : action_base< ACTION_NAME >                     \
    {                                                                   \
        template<typename stack_type>                                   \
            static void apply(const std::string& str, stack_type& s)    \
        {                                                               \
            s.EFFECT();                                                 \
        }                                                               \
    }                                                                   \
        
#define DECLARATION__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT) \
    struct ACTION_NAME : action_base< ACTION_NAME >                     \
    {                                                                   \
        template<typename stack_type>                                   \
            static void apply(const std::string& str, stack_type& s)    \
        {                                                               \
            s.EFFECT(str);                                              \
        }                                                               \
    }                                                                   \


#define UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, FOR_TYPE) \
    template<>                                                          \
    void ACTION_NAME::apply<FOR_TYPE>(const std::string& str,           \
                                             FOR_TYPE& s)               \
    {                                                                   \
        UNRECOVERABLE_ERROR("UNIMPLEMENTED");                           \
    }                                                                   \
    
#define SIMPLE_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT, FOR_TYPE) \
    template<>                                                          \
    void ACTION_NAME::apply<FOR_TYPE>(const std::string& str,           \
                                             FOR_TYPE& s)               \
    {                                                                   \
        s.EFFECT();                                                     \
    }                                                                   \
    
#define FORWARDING_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT, FOR_TYPE) \
    template<>                                                          \
    void ACTION_NAME::apply<FOR_TYPE>(const std::string& str,           \
                                             FOR_TYPE& s)               \
    {                                                                   \
        s.EFFECT(str);                                                  \
    }                                                                   \
    
#define DECLARATION__PEGTL_ACTION(ACTION_NAME)                          \
    struct ACTION_NAME : action_base< ACTION_NAME >                     \
    {                                                                   \
        template<typename stack_type>                                   \
            static void apply(const std::string& str, stack_type& s);   \
    }                                                                   \



#define domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT) \
    SIMPLE_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT, Planning::Parsing::Domain_Data) \
        
#define domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT) \
    FORWARDING_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT, Planning::Parsing::Domain_Data) \

#define domain__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME) \
    UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, Planning::Parsing::Domain_Data) \


#define problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT) \
    SIMPLE_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT, Planning::Parsing::Problem_Data) \
        
#define problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT) \
    FORWARDING_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, EFFECT, Planning::Parsing::Problem_Data) \

#define problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME) \
    UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(ACTION_NAME, Planning::Parsing::Problem_Data) \



#include <pegtl.hh>

#include "dtp_pddl_parsing_data.hh"

namespace Planning
{
    namespace Parsing
    {
        using namespace pegtl;

        struct DEBUG__Action : action_base< DEBUG__Action >
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                std::cerr<<&s<<str<<std::endl;//VERBOSER(1000, str);
                {char ch; std::cin>>ch;}
                
            }
        };

        DECLARATION__PEGTL_ACTION(Start_Effect_Parsing__Action);
        DECLARATION__PEGTL_ACTION(Stop_Effect_Parsing__Action);
        DECLARATION__PEGTL_ACTION(Predicate_Name__Action);
        DECLARATION__PEGTL_ACTION(Action_Name__Action);
        DECLARATION__PEGTL_ACTION(Percept_Name__Action);
        DECLARATION__PEGTL_ACTION(State_Function_Name__Action);
        DECLARATION__PEGTL_ACTION(Perceptual_Function_Name__Action);
        DECLARATION__PEGTL_ACTION(Variable_Argument__Action);
        DECLARATION__PEGTL_ACTION(Type_Of_Argument__Action);
        DECLARATION__PEGTL_ACTION(Dive__Action);
        DECLARATION__PEGTL_ACTION(Emerge__Action);
        DECLARATION__PEGTL_ACTION(Empty_Formula__Action);
        DECLARATION__PEGTL_ACTION(Skip_Next____Formula__Action____Action);
        DECLARATION__PEGTL_ACTION(Formula_Predicate__Action);
        DECLARATION__PEGTL_ACTION(Formula_Action__Action);
        DECLARATION__PEGTL_ACTION(Formula_Percept__Action);
        DECLARATION__PEGTL_ACTION(Formula_State_Function__Action);
        DECLARATION__PEGTL_ACTION(Formula_Perceptual_Function__Action);
        DECLARATION__PEGTL_ACTION(Not__Action);
        DECLARATION__PEGTL_ACTION(And__Action);
        DECLARATION__PEGTL_ACTION(Or__Action);
        DECLARATION__PEGTL_ACTION(If__Action);
        DECLARATION__PEGTL_ACTION(Variable_Cluster__Action);
        DECLARATION__PEGTL_ACTION(Exists__Action);
        DECLARATION__PEGTL_ACTION(Forall__Action);
        DECLARATION__PEGTL_ACTION(Formula__Action);
        DECLARATION__PEGTL_ACTION(Constant_Argument__Action);
        DECLARATION__PEGTL_ACTION(Conditional_Effect__Action);
        DECLARATION__PEGTL_ACTION(Forall_Effect__Action);
        DECLARATION__PEGTL_ACTION(Number__Action);
        DECLARATION__PEGTL_ACTION(Type__Action);
        DECLARATION__PEGTL_ACTION(Type_Of_Type__Action);
        DECLARATION__PEGTL_ACTION(Constant__Action);
        DECLARATION__PEGTL_ACTION(Type_Of_Constant__Action);
        DECLARATION__PEGTL_ACTION(Probabilistic__Action);
        
        DECLARATION__PEGTL_ACTION(Increase__Action);
        DECLARATION__PEGTL_ACTION(Decrease__Action);
        DECLARATION__PEGTL_ACTION(Assign__Action);
        DECLARATION__PEGTL_ACTION(Equality__Action);
        
        DECLARATION__PEGTL_ACTION(GOT_REAL_NUMBER__Action);
        DECLARATION__PEGTL_ACTION(GOT_INTEGER_NUMBER__Action);
        DECLARATION__PEGTL_ACTION(Add_Constants__Action);
        DECLARATION__PEGTL_ACTION(Type_Of_Type__TO__Type_Of_Constant__Action);
        DECLARATION__PEGTL_ACTION(Commit_Constants__Action);
        DECLARATION__PEGTL_ACTION(Domain_Name__Action);


        
        DECLARATION__PEGTL_ACTION(Number_In_Formula__Action);
        DECLARATION__PEGTL_ACTION(Object_In_Formula__Action);
        DECLARATION__PEGTL_ACTION(Constant_In_Formula__Action);
        
        
    }
}
  

#endif
