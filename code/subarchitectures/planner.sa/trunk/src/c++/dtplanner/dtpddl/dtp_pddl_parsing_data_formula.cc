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
#include "dtp_pddl_parsing_data_formula.hh"
#include "dtp_pddl_parsing_data_constants.hh"


#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning::Parsing;

bool Formula_Data::is_static_fluent(const Planning::State_Function_Name& state_Function_Name) const
{
    if(modified_in_effect__state_functions__parsed.find(state_Function_Name)
       == modified_in_effect__state_functions__parsed.end()){
        return true;
    } else if (modified_in_effect__state_ground_functions__parsed.find(state_Function_Name)
               == modified_in_effect__state_ground_functions__parsed.end()) {
        return true;
    }
    
    return false;
}


bool Formula_Data::in_add_effect(const Predicate_Name& predicate_Name) const
{
    if(added__state_predicates__parsed.find(predicate_Name)
       != added__state_predicates__parsed.end()){
        return true;
    }  else if(added__state_propositions__parsed.find(predicate_Name)
               != added__state_propositions__parsed.end()) {
        return true;
    }
    
    return false;
}


bool Formula_Data::in_delete_effect(const Predicate_Name& predicate_Name) const
{
    if(deleted__state_predicates__parsed.find(predicate_Name)
       != deleted__state_predicates__parsed.end()){
        return true;
    } else if(deleted__state_propositions__parsed.find(predicate_Name)
              != deleted__state_propositions__parsed.end()) {
        return true;
    }
    
    return false;
}


void Formula_Data::report__enter_parsing_initial_state()
{
    INTERACTIVE_VERBOSER(true, 3102, "Parsing initial state.");
    parsing_initial_state = true;
}

void Formula_Data::report__exit_parsing_initial_state()
{
    INTERACTIVE_VERBOSER(true, 3102, "Finished parsing initial state.");
    parsing_initial_state = false;
}


Formula_Data::Formula_Data():
    formula_parsing_level(0),
    skip_next____report__formula(false),
    last_number_parsed_was_double(false),
    parsing_initial_state(false),
    in_delete_context(false),
    in_modification_context(false),
    in_effect_context(false)
{
}

void Formula_Data::report__enter_parsing_effect_context()
{
    INTERACTIVE_VERBOSER(true, 3102, "Parsing symbols in effect context.");
    in_effect_context = true;
}

void Formula_Data::report__exit_parsing_effect_context()
{
    INTERACTIVE_VERBOSER(true, 3102, "No longer parsing symbols in effect context.");
    in_effect_context = false;
}


void Formula_Data::report__constant_in_formula()
{
    report__dive();
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        VERBOSER(111, "Preparing space to add constant at level ::"<<(formula_parsing_level)<<std::endl);
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
        VERBOSER(111, ":: DONE ::"<<std::endl);
    }
    
    assert(subformulae.find(formula_parsing_level) != subformulae.end());
    assert(argument_List.size());
    subformulae[formula_parsing_level].push_back(argument_List.back());
    argument_List = Argument_List();
    
    report__emerge();
}

void Formula_Data::report__object_in_formula()
{
    report__dive();
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        VERBOSER(111, "Preparing space to add variable at level ::"<<(formula_parsing_level)<<std::endl);
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
        VERBOSER(111, ":: DONE ::"<<std::endl);
    }
    
    assert(subformulae.find(formula_parsing_level) != subformulae.end());
    assert(argument_List.size());
    assert(argument_List.back().test_cast<Variable>());
    subformulae[formula_parsing_level].push_back(argument_List.back());
    variables.insert(argument_List.back().do_cast_and_copy<Planning::Variable>());
    argument_List = Argument_List();
    
    report__emerge();
}

void Formula_Data::report__number_in_formula()
{
    report__dive();
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        VERBOSER(111, "Preparing space to add number at level ::"<<(formula_parsing_level)<<std::endl);
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
        VERBOSER(111, ":: DONE ::"<<std::endl);
    }
    
    auto probability = last_number__double;
    if(!last_number_parsed_was_double){
        probability = static_cast<double>(last_number__int);
    } 
    
    INTERACTIVE_VERBOSER(true, 10200, "At level :: "<<formula_parsing_level
                         <<" Got number :: "<<probability<<" in action effect.");
    
    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
        (Planning::Formula::Number
         , number
         , probability);
    
    assert(subformulae.find(formula_parsing_level) != subformulae.end());
    subformulae[formula_parsing_level].push_back(number);
    
    report__emerge();
}

void Formula_Data::report__probabilistic_formula()
{
    formula_type.push(probabilistic_effect);
    VERBOSER(111, "(probabilistic NUM (... ) NUM (...)...) at stack element :: "
             <<formula_type.size()<<std::endl);
}

void Formula_Data::report__increase_formula()
{
    formula_type.push(increase);

    VERBOSER(2000, "Got an (increase (... ) NUM) at stack element :: "
             <<formula_type.size()<<std::endl);

    in_modification_context = true;
}

void Formula_Data::report__decrease_formula()
{
    formula_type.push(decrease);

    VERBOSER(2000, "Got an (decrease (... ) NUM) at stack element :: "
             <<formula_type.size()<<std::endl);

    in_modification_context = true;
}

void Formula_Data::report__assign_formula()
{
    formula_type.push(assign);
    
    INTERACTIVE_VERBOSER(true, 10002, "Got an (assign (... ) NUM) at stack element :: "
             <<formula_type.size()<<std::endl);

    in_modification_context = true;
}


void  Formula_Data::report__equality_formula()
{
    if(parsing_initial_state){
        report__assign_formula();
    } else {
        formula_type.push(equality_test);
        
        VERBOSER(2000, "Got an (= (... ) NUM) at stack element :: "
                 <<formula_type.size()<<std::endl);
    }
}


void Formula_Data::report__skip_next____report__formula()
{
    VERBOSER(41, "Skipping on :: "<<formula_parsing_level);
    skip_next____report__formula = true;
}

void Formula_Data::report__dive(){;
    formula_parsing_level++;
    assert(formula_parsing_level>=0);
    VERBOSER(111, "Got a parenthesis '(' while parsing formula."<<std::endl
             <<"formula_parsing_level is now :: "<<formula_parsing_level<<std::endl)
}
void Formula_Data::report__emerge(){
    formula_parsing_level--;
    assert(formula_parsing_level>=0);
    VERBOSER(111, "Got a parenthesis ')' while parsing formula."
             <<"formula_parsing_level is now :: "<<formula_parsing_level<<std::endl);
}
            

void Formula_Data::report__empty_formula()
{
    VERBOSER(111, "Parsed empty formula.");
    formula_type.push(vacuous);
}

void Formula_Data::report__if_formula()
{
    formula_type.push(material_implication);
}

void Formula_Data::report__not_formula()
{
    formula_type.push(negation);

    /* delete context is STARTED here. it is stopped when this parsed
     * symbol is delt with in \member{report__formula()}
     * \case{negation}. */
    in_delete_context = true;
}

void Formula_Data::report__and_formula()
{
    VERBOSER(22, "Parsing ::CONJUNCT::AND:: at level :: "<<formula_parsing_level<<std::endl);
    formula_type.push(conjunction);
}

void Formula_Data::report__or_formula()
{
    formula_type.push(disjunction);
}

void Formula_Data::report__exists_formula()
{
    formula_type.push(exists);
}

void Formula_Data::report__forall_formula()
{
    formula_type.push(forall);
}

void Formula_Data::report__conditional_effect_formula()
{
    formula_type.push(conditional_effect);
}

Planning::Formula::Subformula
Formula_Data::complete__probabilistic_formula()
{
    assert(subformulae[formula_parsing_level+1].size());
    
    {/*BEGIN -- In case of parsing or authorship errors.*/
        
        std::ostringstream oss;
        for(auto p = subformulae[formula_parsing_level+1].begin()
                ; p != subformulae[formula_parsing_level+1].end()
                ; p++){
            oss<<*p<<" ";
        }
        std::string str = oss.str();
        
        QUERY_UNRECOVERABLE_ERROR(
            subformulae[formula_parsing_level+1].size() % 2,/*Index starting at 0*/
            "While parsing (:probabilistic num (formula) num (formula)) element."<<std::endl
            <<"Expecting something of the above form, but got :: "<<str<<std::endl
            <<"Which has :: "<<subformulae[formula_parsing_level+1].size()<<" elements."<<std::endl);
    }/*END -- In case of parsing or authorship errors.*/

    //Planning::Formula::numbers__vector;
    Formula::Subformulae probabilities;
    Planning::Formula::Subformulae associated_formula;
    
    for(int i = 0
            ; i < subformulae[formula_parsing_level+1].size()
            ; i++){
        auto tmp = subformulae[formula_parsing_level+1][i]; 
        if(!(i%2)){

            switch(tmp->get__type_name()){
                case enum_types::number:
                break;
                case enum_types::state_ground_function:
                break;
                case enum_types::state_function:
                break;
                case enum_types::perceptual_ground_function:
                break;
                case enum_types::perceptual_function:
                break;
                case enum_types::state_proposition:
                {   
                    /* Accidentally decided a ground state function was
                     * a perceptual proposition. So here we do the
                     * type translation.*/
                    
                    assert(tmp.test_cast<Formula::State_Proposition>());
                    auto state_Proposition = tmp.cxx_get<Formula::State_Proposition>();
                    
                    
                    /*Turn this perceptual proposition into a ground state function.*/
                    auto& name = state_Proposition->get__name();
                    auto& arguments = state_Proposition->get__arguments();

                    
                    NEW_object_referenced_WRAPPED//_deref_visitable_POINTER
                        (State_Function_Name
                         , _name
                         , name.get__name()
                         );
                    

                    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                        (Formula::State_Ground_Function
                         , _tmp
                         , _name
                         , arguments);
                    
                    tmp = _tmp;
                    
                }
                break;
                case enum_types::perceptual_proposition:
                {
                    
                    /* Accidentally decided a ground state function was
                     * a perceptual proposition. So here we do the
                     * type translation.*/
                    
                    assert(tmp.test_cast<Formula::Perceptual_Proposition>());
                    auto perceptual_Proposition = tmp.cxx_get<Formula::Perceptual_Proposition>();
                    
                    
                    /*Turn this perceptual proposition into a ground state function.*/
                    auto& name = perceptual_Proposition->get__name();
                    auto& arguments = perceptual_Proposition->get__arguments();

                    
                    NEW_object_referenced_WRAPPED//_deref_visitable_POINTER
                        (State_Function_Name
                         , _name
                         , name.get__name()
                         );
                    

                    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                        (Formula::State_Ground_Function
                         , _tmp
                         , _name
                         , arguments);
                    
                    tmp = _tmp;
                    
                }
                break;
                default:
                    UNRECOVERABLE_ERROR("Expecting a number, but got :: "<<tmp<<" of type "
                                        <<tmp->get__type_name()<<std::endl);
                    break;
            }
            
            probabilities.push_back(tmp);//.do_cast_and_copy<Planning::Formula::Number>());
        } else {
            associated_formula.push_back(tmp);
        }
    }

    
    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
        (Planning::Formula::Probabilistic
         , tmp
         , associated_formula
         , probabilities);

    
    return tmp;
}


Planning::Formula::Subformula
Formula_Data::complete__quantified_formula(int quantifier)
{
   
    QUERY_UNRECOVERABLE_ERROR(
        !stack_of__Typed_Arguments.size(),
        "Parsing a quantifier symbol -- e.g. (forall (?a - TYPE ) (formula)) --"<<std::endl
        <<"but no variables -- e.g. (?a - TYPE) -- seem to have been specified.");
    
    
    /*Specification of arguments that were quantified.*/
    auto __quantified_variables = stack_of__Typed_Arguments.top();
    stack_of__Typed_Arguments.pop();/*Using some parsed variable arguments.*/



    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(formula_parsing_level + 1) == subformulae.end(),
        "Got a quantified varaibles :: "<<__quantified_variables<<" ::  without a subformula.");
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[formula_parsing_level + 1].size() != 1,
        "Got a quantified variables :: "<<__quantified_variables<<" :: without a coherent subformula.");
    
    Planning::Argument_List _quantified_variables
        = Planning::get__symbols(__quantified_variables);
    
//     Planning::Argument_Types variables_types
//         = Planning::get__types(__quantified_variables);

    /* Set of variables that were quantified. Converting vector of
     * pointers to variables to vector of variables. Then storing the
     * result as a set of variables \local{_variables}.*/
    Variables _variables;
    {
        std::vector<Variable> quantified_variables(_quantified_variables.size());
        
        uint i = 0;
        for(auto var = _quantified_variables.begin()
                ; var != _quantified_variables.end()
                ; var++){
            assert(var->test_cast<Variable>());
            assert(i < quantified_variables.size());
            quantified_variables[i++] = var->do_cast_and_copy<Variable>();
        }
        
        /* Variable symbols that this quantifier is talking about.*/
         _variables = Variables(quantified_variables.begin(),
                                quantified_variables.end());
    }

    /* We are making a derived predicate for the quantifier here. The
     * name of that predicate will be given in terms of the quantified
     * formula.*/
    assert(subformulae.find(formula_parsing_level + 1) != subformulae.end());
    assert(subformulae[formula_parsing_level + 1].size());
    std::ostringstream oss;
    oss<<"DERIVED-PREDICATE-"
       <<subformulae[formula_parsing_level + 1][0]->get__type_name()
       <<"-"
       <<quantifier
       <<"-"
       <<subformulae[formula_parsing_level + 1][0]->get__id();
    NEW_object_referenced_WRAPPED(Planning::Predicate_Name,
                                  name,
                                  oss.str());
    
    for(auto variable = _variables.begin()
            ; variable != _variables.end()
            ; variable++){
        /*This no longer occurs free in the subformula.*/
        this->variables.erase(*variable);
    }
    
    Typed_Arguments derived_predicatex_arguments;
    /*For each variable that was parsed in a subformula.*/
    for(auto variable = this->variables.begin()
            ; variable != this->variables.end()
            ; variable++){
        auto some_types = find__type_of_variable(*variable);

        QUERY_UNRECOVERABLE_ERROR(
            !some_types.size(),
            "Could not find types for variable :: "<<*variable<<std::endl
            <<"While parsing quantifier for :: "<<subformulae[formula_parsing_level + 1].back()<<std::endl);
        
        NEW_object_referenced_WRAPPED_deref_visitable_POINTER
            (Planning::Variable, new_variable, variable->get__name());
        Planning::get__symbols(derived_predicatex_arguments).push_back(new_variable);
        Planning::get__types(derived_predicatex_arguments).push_back(some_types);
    }
    
    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
        (Planning::Derived_Predicate
         , new_derived_predicate
         , name
         , derived_predicatex_arguments
         , __quantified_variables
         , _variables
         , subformulae[formula_parsing_level + 1].back()
         , quantifier
//          , -1
         );
    
    VERBOSER(41, "GOT :: "<<new_derived_predicate<<std::endl);

    derived_Predicates__artificial
        .insert(*dynamic_cast<Planning::Derived_Predicate*>
                (new_derived_predicate.cxx_get().get()));
    
    return new_derived_predicate; 
}

Planning::Formula::Subformula
Formula_Data::complete__forall_formula()
{
    return complete__quantified_formula(Planning::enum_types::forall);
}

Planning::Formula::Subformula
Formula_Data::complete__exists_formula()
{
    return complete__quantified_formula(Planning::enum_types::exists);
}










// template<typename PROPOSITION_SYMBOL, typename PREDICATE_SYMBOL>
// report__formula_atomic_symbol<PROPOSITION_SYMBOL, PREDICATE_SYMBOL>()


void  Formula_Data::report__formula_action()
{
    report__formula_atomic_symbol
        <Planning::Formula::Action_Proposition
        , Planning::Formula::Action_Predicate
        , Planning::Action_Name>
        (action_Name);
}

void  Formula_Data::report__formula_percept()
{
    report__formula_atomic_symbol
        <Planning::Formula::Perceptual_Proposition
        , Planning::Formula::Perceptual_Predicate
        , Planning::Percept_Name>
        (percept_Name);
}

void  Formula_Data::report__formula_perceptual_function()
{
    report__formula_atomic_symbol
        <Planning::Formula::Perceptual_Ground_Function
        , Planning::Formula::Perceptual_Function
        , Planning::Perceptual_Function_Name>
        (perceptual_Function_Name);
}

void  Formula_Data::report__formula_state_function()
{
    INTERACTIVE_VERBOSER(true, 10002, "Parsing state function :: "<<state_Function_Name);
    report__formula_atomic_symbol
        <Planning::Formula::State_Ground_Function
        , Planning::Formula::State_Function
        , Planning::State_Function_Name>
        (state_Function_Name);
}

void  Formula_Data::report__formula_predicate()
{
    report__formula_atomic_symbol
        <Planning::Formula::State_Proposition
        , Planning::Formula::State_Predicate
        , Planning::Predicate_Name>
        (predicate_Name);
}

void Formula_Data::report__formula(const std::string& str)
{
    if(skip_next____report__formula){
        INTERACTIVE_VERBOSER(true, 10001,"Skipping off :: "<<formula_parsing_level<<" :: "<<str<<std::endl);
        skip_next____report__formula = false;
        return;
    }
    
    
    QUERY_UNRECOVERABLE_ERROR(!formula_type.size(), "Failed to get formula type from :: "
                              <<formula_parsing_level<<" :: "<<str<<std::endl
                              <<"Could be a misspelling, or I stuffed up the operator set"<<std::endl
                              <<"supported by my parser.");
    
    VERBOSER(31, "Got :: "<<formula_type.top()<<std::endl
             <<"From :: "<<str<<std::endl);

    QUERY_UNRECOVERABLE_ERROR
        (formula_parsing_level < 0,
         "We are parsing a formula at level :: "<<formula_parsing_level<<" :: "
         <<str<<" :: I can't accept a negative parse level!"<<std::endl);
    
    
    if(subformulae.find(formula_parsing_level) == subformulae.end()){
        subformulae[formula_parsing_level] = Planning::Formula::Subformulae();
    }
            
    switch(formula_type.top()){
        case vacuous:
        {
            VERBOSER(25, "VACUOUS");
            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Vacuous
                 , tmp
                 , static_cast<void*>(0));
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case forall:
        {
            VERBOSER(25, "FORALL");
            auto tmp = complete__forall_formula();
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case exists:
        {
            auto tmp = complete__exists_formula();
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;            
        case disjunction:
        {
            VERBOSER(25, "disjunction");

            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
    
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Disjunction
                 , tmp
                 , subformulae[formula_parsing_level+1]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
            break;
        case conjunction:
        {
            VERBOSER(25, "conjunction");

//             assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            
            if(!check__exists_parsed_subformulae(formula_parsing_level + 1)){
                
                NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                    (Planning::Formula::Conjunction
                     , tmp
                     , Formula::Subformulae());
                subformulae[formula_parsing_level].push_back(tmp);
            } else {
                
                NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                    (Planning::Formula::Conjunction
                     , tmp
                     , subformulae[formula_parsing_level + 1]);
                subformulae[formula_parsing_level].push_back(tmp);
            }
            
            
        }
        break;
        case negation:
        {
            /* END of the delete context (started in \member{report__not_formula()}).*/
            in_delete_context = false;
            
            VERBOSER(25, "negation");
            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            
            assert(subformulae[formula_parsing_level+1].size() == 1);
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Negation
                 , tmp
                 , subformulae[formula_parsing_level+1][0]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case material_implication:
        {
            VERBOSER(25, "material_implication");
            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            check__cardinality_constraint_on_subformulae_at_index(2, formula_parsing_level + 1);
            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Negation
                 , tmp1
                 , subformulae[formula_parsing_level + 1][0]);
            
            subformulae[formula_parsing_level + 1][0] = tmp1;
            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Disjunction
                 , tmp
                 , subformulae[formula_parsing_level + 1]);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case increase:
        {
            in_modification_context = false;
            
            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);
            
            
            
            auto subs = subformulae[formula_parsing_level+1].begin();
            auto evaluation_expression_LHS = *subs;
            subs++;
            auto evaluation_expression_RHS = *subs;

            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Increase
                 , tmp
                 , evaluation_expression_LHS
                 , evaluation_expression_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case decrease:
        {
            in_modification_context = false;
            
            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);
            
            auto subs = subformulae[formula_parsing_level+1].begin();
            auto evaluation_expression_LHS = *subs;
            subs++;
            auto evaluation_expression_RHS = *subs;

            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Decrease
                 , tmp
                 , evaluation_expression_LHS
                 , evaluation_expression_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case assign:
        {
            in_modification_context = false;

            if(parsing_initial_state){
                INTERACTIVE_VERBOSER(true, 18000, "Parsing an assignment in the initial state.");

                if(1 == formula_parsing_level){
                    INTERACTIVE_VERBOSER(true, 18000, "Parsing level is at 2.");
                } else {
                    INTERACTIVE_VERBOSER(true, 18000, "Parsing level is at :: "<<formula_parsing_level<<std::endl);
                }
                
                
            }
            
            
            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);
            
            auto subs = subformulae[formula_parsing_level+1].begin();
            auto evaluation_expression_LHS = *subs;
            subs++;
            auto evaluation_expression_RHS = *subs;

            if( parsing_initial_state &&
                2 == formula_parsing_level &&
                enum_types::number == evaluation_expression_RHS->get__type_name() ){

                INTERACTIVE_VERBOSER(true, 18000, "Parsing number in initial state :: "<<evaluation_expression_RHS);
                
                if(evaluation_expression_LHS.test_cast<Formula::State_Ground_Function>()){
                
                    auto index = *evaluation_expression_LHS
                        .CXX__deref__shared_ptr<basic_type>::cxx_get<Formula::State_Ground_Function>();
                
                    auto value = evaluation_expression_RHS.cxx_get<Formula::Number>()->get__value();

                    /* We are already parsing an initial state, so we
                     * must be a problem description. Therefore we can
                     * make that cast. Then we can extract the domain,
                     * and do type checking on the function names.  */
                    auto problem_Data = dynamic_cast<Problem_Data*>(this);
                    auto domain_Data = problem_Data->get__domain_Data();
                    //auto domain_formula_data = dynamic_cast<Formula_Data*>(domain_Data);
                    
                    if(domain_Data->is_type__double(index.get__name())){
                        INTERACTIVE_VERBOSER(true, 18000, "Static DOUBLE assignment for :: "
                                             <<"(assign "<<evaluation_expression_LHS
                                             <<" "<<evaluation_expression_RHS<<std::endl);
                        static_ground_double_function[index] = value;
                    }  else if (domain_Data->is_type__int(index.get__name())
                                || domain_Data->is_type__number(index.get__name())) {
                        
                        INTERACTIVE_VERBOSER(true, 18000, "Static INT assignment for :: "
                                             <<"(assign "<<evaluation_expression_LHS
                                             <<" "<<evaluation_expression_RHS<<std::endl);
                        static_ground_int_function[index] = static_cast<int>(value);
                    } else {
                        UNRECOVERABLE_ERROR("Assignment to non-int and non-double.");
                    }
                    
                }
            } else if (parsing_initial_state) {
                
                INTERACTIVE_VERBOSER(true, 5000, "In initial state.. Non-static assignment for :: "
                                     <<"(assign "<<evaluation_expression_LHS
                                     <<" "<<evaluation_expression_RHS<<std::endl);
            }
            
            
            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Assign
                 , tmp
                 , evaluation_expression_LHS
                 , evaluation_expression_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        case equality_test:
        {
            UNRECOVERABLE_ERROR("unimplemented.");
        }
        break;
        case conditional_effect:
        {
            /* (when (boolean-condition/i.e., precondition)
             * (effect-formula)). As with LAMA, this is to be later
             * treated as an action that is necessarily triggered by
             * the parent effect/action.*/

            
            assert(check__exists_parsed_subformulae(formula_parsing_level + 1));
            check__cardinality_constraint_on_subformulae_at_index
                (2, formula_parsing_level+1);


            auto components = subformulae[formula_parsing_level+1].begin();
            auto precondition_LHS = *components;
            components++;
            auto effect_RHS = *components;            
            
            NEW_object_referenced_WRAPPED_deref_visitable_POINTER
                (Planning::Formula::Conditional_Effect
                 , tmp
                 , precondition_LHS
                 , effect_RHS);
            
            subformulae[formula_parsing_level].push_back(tmp);
            
        }
        break;
        case probabilistic_effect:
        {
            auto tmp = complete__probabilistic_formula();

            
            INTERACTIVE_VERBOSER(true, 4110, "Got new probabilistic formula :: "<<tmp);
            subformulae[formula_parsing_level].push_back(tmp);
        }
        break;
        default:
            UNRECOVERABLE_ERROR("Unable to parse formula.");
            break;
    }

    /*RESET subformulae.*/
    subformulae[formula_parsing_level + 1] = Planning::Formula::Subformulae();

    if(formula_parsing_level == 1){
        assert(subformulae.find(formula_parsing_level) != subformulae.end());
        assert(subformulae[formula_parsing_level].size());
        VERBOSER(111, "FORMULA :: "<<*subformulae[formula_parsing_level].begin()<<std::endl);
    }
    
    formula_type.pop();
}




void Formula_Data::commit__argument_types()
{
    /*New variables*/
    for(auto i = argument_List.begin(); i != argument_List.end(); i++){
        std::tr1::get<0>(typed_Arguments).push_back(*i);
    }

    Types object_types;
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory, Planning::Type, object_type, "object");
        object_types.insert(object_type);
    } else {
        NEW_object_referenced_WRAPPED(Planning::Type, object_type, "object");
        object_types.insert(object_type);
    }
    
//     NEW_object_referenced_WRAPPED(Planning::Type, object_type, "object");
//     object_types.insert(object_type);
    
    /*And their types.*/
    for(auto i = argument_List.begin(); i != argument_List.end(); i++){

        //         auto q = i->get().get();
        //         dynamic_cast<Variable*>(q);
        
        if(i->test_cast<Variable>()){//dynamic_cast<Variable*>(i->get())){
            if(types_of_types.size()){    
                std::tr1::get<1>(typed_Arguments).push_back(types_of_types);
            } else { 
                std::tr1::get<1>(typed_Arguments).push_back(object_types);
            }
        } else {
            /* I may-as-well suppose all constant arguments to a
             *  predicate symbol have the type "object".*/
            std::tr1::get<1>(typed_Arguments).push_back(object_types);
        }   
    }
    
    
    argument_List = Argument_List ();
    types_of_types = Types();
}


void Formula_Data::add__number (const std::string& str)
{
    VERBOSER(111, "Parsing a number :: "<<str<<std::endl);
    
    std::istringstream iss(str);
    if(last_number_parsed_was_double){
        iss>>last_number__double;
        INTERACTIVE_VERBOSER(true, 10200, "This was a double with value :: "<<last_number__double<<std::endl);
    } else  {
        iss>>last_number__int;
        INTERACTIVE_VERBOSER(true, 10200, "This was an int with value :: "<<last_number__int<<std::endl);
    }
}

void Formula_Data::report__parsing_real_number()
{
    last_number_parsed_was_double = true;
}


void Formula_Data::report__parsing_integer_number()
{
    last_number_parsed_was_double = false;
}

void Formula_Data::add__variable_argument(const std::string& str)
{
    NEW_object_referenced_WRAPPED_deref_visitable_POINTER(Planning::Variable, variable, str);
    
    argument_List.push_back(variable);
}

void Formula_Data::add__constant_argument(const std::string& str)
{
    assert(dynamic_cast<Constants_Data*>(this));
//     NEW_object_referenced_WRAPPED_deref_visitable_POINTER(Planning::Constant, constant, str);

    INTERACTIVE_VERBOSER(true, 10003, "Pointers from formula are "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(dynamic_cast<Constants_Data*>(this))
                         <<" "<<reinterpret_cast<basic_type::Runtime_Thread>(this));
    NEW_referenced_WRAPPED_deref_visitable_POINTER(dynamic_cast<Constants_Data*>(this), Planning::Constant, constant, str);

    argument_List.push_back(constant);
}



#define REPORT_SYMBOL_USAGE__PROPOSITIONAL(TYPE, NAME_TYPE, storage, add_storage, del_storage, OTHER_STUFF) \
    template<>                                                          \
    void Formula_Data::using__symbol_name<TYPE                          \
                                          , NAME_TYPE>                  \
    (const NAME_TYPE& symbol_name,                                      \
     ID_TYPE index)                                                     \
    {                                                                   \
        INTERACTIVE_VERBOSER(true, 3102, "Processing symbol in a ("<<in_effect_context<<")context :: "<<symbol_name); \
                                                                        \
        if(storage.find(symbol_name) == storage.end()){                 \
            storage[symbol_name] = std::set<ID_TYPE>();                 \
        }                                                               \
        storage[symbol_name]                                            \
            .insert(index);                                             \
                                                                        \
        OTHER_STUFF;                                                    \
                                                                        \
        if(in_effect_context){                                          \
INTERACTIVE_VERBOSER(true, 3102, "Parsing symbol in effect context :: "<<symbol_name); \
            if(in_delete_context){                                      \
                if(del_storage.find(symbol_name) == del_storage.end()){ \
                    del_storage[symbol_name] = std::set<ID_TYPE>();     \
                }                                                       \
INTERACTIVE_VERBOSER(true, 3102, "Got an delete symbol :: "<<symbol_name); \
                del_storage[symbol_name]                                \
                    .insert(index);                                     \
            } else {                                                    \
                                                                        \
                if(add_storage.find(symbol_name) == add_storage.end()){ \
                    add_storage[symbol_name] = std::set<ID_TYPE>();     \
                }                                                       \
                add_storage[symbol_name]                                \
                    .insert(index);                                     \
                                                                        \
INTERACTIVE_VERBOSER(true, 3102, "Got an add symbol :: "<<symbol_name); \
            }                                                           \
        }                                                               \
    }                                                                   \

#define REPORT_SYMBOL_USAGE__FUNCTIONAL(TYPE, NAME_TYPE, storage, mod_storage) \
    template<>                                                          \
    void Formula_Data::using__symbol_name<TYPE                          \
                                          , NAME_TYPE>                  \
    (const NAME_TYPE& symbol_name,                                      \
     ID_TYPE index)                                                     \
    {                                                                   \
        if(storage.find(symbol_name) == storage.end()){                 \
            storage[symbol_name] = std::set<ID_TYPE>();                 \
        }                                                               \
        storage[symbol_name]                                            \
            .insert(index);                                             \
                                                                        \
        if(in_modification_context){                                    \
                                                                        \
            if(mod_storage.find(symbol_name) == mod_storage.end()){     \
                mod_storage[symbol_name] = std::set<ID_TYPE>();         \
            }                                                           \
            mod_storage[symbol_name].insert(index);                     \
        }                                                               \
                                                                        \
    }                                                                   \


#define REPORT_SYMBOL_USAGE__NO_STORAGE(TYPE, NAME_TYPE) template<>     \
    void Formula_Data::using__symbol_name<TYPE                          \
                                          , NAME_TYPE>                  \
    (const NAME_TYPE& symbol_name,                                      \
     ID_TYPE index)                                                     \
    {                                                                   \
    }                                                                   \



namespace Planning
{
    namespace Parsing
    {
        
        REPORT_SYMBOL_USAGE__FUNCTIONAL(Planning::Formula::Perceptual_Function,
                                        Planning::Perceptual_Function_Name,
                                        perceptual_functions__parsed,
                                        modified_in_effect__perceptual_functions__parsed);
        
        REPORT_SYMBOL_USAGE__FUNCTIONAL(Planning::Formula::State_Function,
                                        Planning::State_Function_Name,
                                        state_functions__parsed,
                                        modified_in_effect__state_functions__parsed);
        
        REPORT_SYMBOL_USAGE__FUNCTIONAL(Planning::Formula::Perceptual_Ground_Function,
                                        Planning::Perceptual_Function_Name,
                                        perceptual_ground_functions__parsed,
                                        modified_in_effect__perceptual_ground_functions__parsed);
        
        REPORT_SYMBOL_USAGE__FUNCTIONAL(Planning::Formula::State_Ground_Function,
                                        Planning::State_Function_Name,
                                        state_ground_functions__parsed,
                                        modified_in_effect__state_ground_functions__parsed);
        
#define STATING_STATE_PROPOSITIONS_TEST                                 \
        {                                                               \
            if( parsing_initial_state &&                                \
                1 == formula_parsing_level)                             \
            {                                                           \
                starting_state_propositions.insert(index);              \
            }                                                           \
        }                                                               \
                                                                        \

#define EMPTY_TEST ;
        
        REPORT_SYMBOL_USAGE__PROPOSITIONAL(Planning::Formula::State_Predicate,
                                           Planning::Predicate_Name,
                                           state_predicates__parsed,
                                           added__state_predicates__parsed,
                                           deleted__state_predicates__parsed,
                                           EMPTY_TEST);
        
        REPORT_SYMBOL_USAGE__PROPOSITIONAL(Planning::Formula::State_Proposition,
                                           Planning::Predicate_Name,
                                           state_propositions__parsed,
                                           added__state_propositions__parsed,
                                           deleted__state_propositions__parsed,
                                           STATING_STATE_PROPOSITIONS_TEST);
        
        REPORT_SYMBOL_USAGE__PROPOSITIONAL(Planning::Formula::Perceptual_Predicate,
                                           Planning::Percept_Name,
                                           perceptual_predicates__parsed,
                                           added__perceptual_predicates__parsed,
                                           deleted__perceptual_predicates__parsed,
                                           EMPTY_TEST);
        
        REPORT_SYMBOL_USAGE__PROPOSITIONAL(Planning::Formula::Perceptual_Proposition,
                                           Planning::Percept_Name,
                                           perceptual_propositions__parsed,
                                           added__perceptual_propositions__parsed,
                                           deleted__perceptual_propositions__parsed,
                                           EMPTY_TEST);

        REPORT_SYMBOL_USAGE__NO_STORAGE(Planning::Formula::Action_Proposition,
                                        Planning::Action_Name);
        
        REPORT_SYMBOL_USAGE__NO_STORAGE(Planning::Formula::Action_Predicate,
                                        Planning::Action_Name);
    }
}


void Formula_Data::report__perceptual_function_name(const std::string& str)
{
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory, Planning::Perceptual_Function_Name, tmp, str);
        perceptual_Function_Name = tmp;
    } else {    
        NEW_object_referenced_WRAPPED(Planning::Perceptual_Function_Name, tmp, str);
        perceptual_Function_Name = tmp;
    }
}

void Formula_Data::report__percept_name(const std::string& str)
{
    //NEW_object_referenced_WRAPPED(Planning::Percept_Name, tmp, str);
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory, Planning::Percept_Name, tmp, str);
        percept_Name = tmp;
    } else {
        NEW_object_referenced_WRAPPED(Planning::Percept_Name, tmp, str);
        percept_Name = tmp;
    }
    
    
//     NEW_object_referenced_WRAPPED(Planning::Percept_Name, tmp, str);
//     percept_Name = tmp;
}

void Formula_Data::report__state_function_name(const std::string& str)
{
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory, Planning::State_Function_Name, tmp, str);
        state_Function_Name = tmp;
    } else {    
        NEW_object_referenced_WRAPPED(Planning::State_Function_Name, tmp, str);
        state_Function_Name = tmp;
    }
    
}

void Formula_Data::report__predicate_name(const std::string& str)
{
    
    if(symbol_theory){
        NEW_referenced_WRAPPED(symbol_theory, Planning::Predicate_Name, tmp, str);
        predicate_Name = tmp;
    } else {    
        NEW_object_referenced_WRAPPED(Planning::Predicate_Name, tmp, str);
        predicate_Name = tmp;
    }
    
}







void Formula_Data::stack__typed_Arguments()
{
    VERBOSER(41, "Pushing back quantified arguments :: "<<typed_Arguments);
    stack_of__Typed_Arguments.push(typed_Arguments);
    typed_Arguments = Typed_Arguments();
}

bool Formula_Data::check__exists_parsed_subformulae(int index) const
{
    if(subformulae.find(index) == subformulae.end()){
        return false;
    }
    
    
    QUERY_UNRECOVERABLE_ERROR
        (subformulae.find(index) == subformulae.end(),
         "Parsing an expression that requires a subformula :: "
         <<"at level :: "<<formula_parsing_level<<std::endl);
    
    return subformulae.find(index) != subformulae.end();
}

bool Formula_Data::check__cardinality_constraint_on_subformulae_at_index
(int count, int index) const
{
    check__exists_parsed_subformulae(index);

    assert(count >= 0);
    if(subformulae.find(index)->second.size() != static_cast<uint>(count)){
        std::ostringstream oss;
        for(auto f = subformulae.find(formula_parsing_level+1)->second.begin()
                ; f != subformulae.find(formula_parsing_level+1)->second.end()
                ; f++){
            oss<<*f;
        }
        
        std::string str = oss.str();
        
        UNRECOVERABLE_ERROR("Expecting :: "<<count
                            <<" subformula for domain element\n"
                            <<"but got :: "<<subformulae.find(formula_parsing_level+1)->second.size()<<std::endl
                            <<"These were :: "<<str<<std::endl);
        
        return false;
    }/* END DEBUG */
    
    return true;
}


bool Formula_Data::potential_match_via_an_assignment(const Planning::Formula::State_Predicate& state_Predicate,
                                                     const Planning::Predicate_Name& proposition_name,
                                                     const Constant_Arguments& propositionx_arguments) const
{
    auto predicate_Name = state_Predicate.get__name();
    
    QUERY_UNRECOVERABLE_ERROR(predicate_Name != proposition_name,
                              "Asked to see if predicate :: "<<state_Predicate<<std::endl
                              <<"Can be ground to :: ("<<proposition_name<<" "<<propositionx_arguments<<")"<<std::endl
                              <<"However, those symbols have inconsistent names.");
    
//     auto propositionx_arguments = state_Proposition.get__arguments();
    auto predicatex_arguments = state_Predicate.get__arguments();
    
    QUERY_UNRECOVERABLE_ERROR(propositionx_arguments.size() != predicatex_arguments.size(),
                              "Asked to see if predicate :: "<<state_Predicate<<std::endl
                              <<"Can be ground to :: ("<<proposition_name<<" "<<propositionx_arguments<<")"<<std::endl
                              <<"However, those symbols have inconsistent arities.");
    
    for(uint index = 0; index <  propositionx_arguments.size(); index++){
        const Constant& prop_arg = propositionx_arguments[index];
        const Formula::Subformula& _pred_arg = predicatex_arguments[index];
        
        
        if(_pred_arg.test_cast<Planning::Constant>()){
            const Constant& pred_arg = *_pred_arg.cxx_get<Constant>();
            if(pred_arg.get__runtime_Thread() == prop_arg.get__runtime_Thread()){
                if(pred_arg == prop_arg) {
                    INTERACTIVE_VERBOSER(true, 3101, "Successful integer match "<<index<<" "<<state_Predicate
                                         <<" ("<<proposition_name<<" "<<propositionx_arguments<<") "<<std::endl);
                    continue;
                } else {
                    INTERACTIVE_VERBOSER(true, 3101, "FAILED integer match "<<index<<" "<<state_Predicate
                                         <<" ("<<proposition_name<<" "<<propositionx_arguments<<") "<<std::endl);
                    return false;
                }
            } else {
                QUERY_WARNING(pred_arg.get__name() == prop_arg.get__name(),
                              "Constant :: "<<pred_arg<<" has thread :: "
                              <<pred_arg.get__runtime_Thread()<<std::endl
                              <<"Whereas Constant :: "<< prop_arg
                              <<" has thread :: "<<prop_arg.get__runtime_Thread()<<std::endl);
                
                if(pred_arg.get__name() == prop_arg.get__name()){
                    INTERACTIVE_VERBOSER(true, 3101, "Successful string match "<<index<<" "<<state_Predicate
                                         <<" ("<<proposition_name<<" "<<propositionx_arguments<<") "<<std::endl);
                    continue;
                } else {
                    INTERACTIVE_VERBOSER(true, 3101, "FAILED string match "<<index<<" "<<state_Predicate
                                         <<" ("<<proposition_name<<" "<<propositionx_arguments<<") "<<std::endl);
                    return false;
                }
            }
        }
    }
    
    INTERACTIVE_VERBOSER(true, 3101, "Got a match "<<state_Predicate
             <<" ("<<proposition_name<<" "<<propositionx_arguments<<") "<<std::endl);
    
    return true;
}


// bool Formula_Data::necessarily_satisfiable(const Planning::Formula::State_Predicate& state_Predicate) const
// {
    
//     auto predicate_Name = state_Predicate.get__name();

//     auto _occurrence_indices = state_propositions__parsed.find(predicate_Name);
//     if(_occurrence_indices == state_propositions__parsed.end()) {
        
//         INTERACTIVE_VERBOSER(true, 3101, "No ground instances of symbol "<<state_Predicate<<std::endl);
        
//         return false;
//     }
    

//     INTERACTIVE_VERBOSER(true, 3101, "Testing ground instances of symbol "<<state_Predicate<<std::endl);
    
//     auto occurrence_indices = _occurrence_indices->second;

//     for(auto index = occurrence_indices.begin()
//             ; index != occurrence_indices.end()
//             ; index++){

//         auto indexed__Traversable_Collection = Planning::Formula::State_Proposition::indexed__Traversable_Collection;
//         auto runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>(this);
        
//         QUERY_UNRECOVERABLE_ERROR(indexed__Traversable_Collection.find(runtime_Thread)
//                                   == indexed__Traversable_Collection.end(),
//                                   "There were not propositions parsed in this descriptive element.");
        
//         auto traversable_Collection = indexed__Traversable_Collection[runtime_Thread];

//         QUERY_UNRECOVERABLE_ERROR(!traversable_Collection.use_count(),
//                                   "There were not propositions parsed in this descriptive element.");
        
//         QUERY_UNRECOVERABLE_ERROR(!(*index < (*traversable_Collection).size()),
//                                   "We parsed :: "<<(*traversable_Collection).size()<<" elements"<<std::endl
//                                   <<"And are asking for element :: "<<*index<<std::endl;);
        
        
//         auto proposition = (*traversable_Collection)[*index];
//         if(potential_match_via_an_assignment(state_Predicate,
//                                              std::tr1::get<0>(proposition),
//                                              std::tr1::get<1>(proposition))) return true;
//     }
    
//     INTERACTIVE_VERBOSER(true, 3101, "No ground instances to match "<<state_Predicate<<std::endl);
    
//     return false;
// }

bool Formula_Data::statically_satisfiable(const Planning::Formula::State_Predicate& state_Predicate) const
{
    
    auto predicate_Name = state_Predicate.get__name();

    auto _occurrence_indices = state_propositions__parsed.find(predicate_Name);
    if(_occurrence_indices == state_propositions__parsed.end()) {
        
        INTERACTIVE_VERBOSER(true, 3101, "No ground instances of symbol "<<state_Predicate<<std::endl);
        
        return false;
    }
    

    INTERACTIVE_VERBOSER(true, 3101, "Testing ground instances of symbol "<<state_Predicate<<std::endl);
    
    auto occurrence_indices = _occurrence_indices->second;

    for(auto index = occurrence_indices.begin()
            ; index != occurrence_indices.end()
            ; index++){

        auto indexed__Traversable_Collection = Planning::Formula::State_Proposition::indexed__Traversable_Collection;
        auto runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>(this);
        
        QUERY_UNRECOVERABLE_ERROR(indexed__Traversable_Collection.find(runtime_Thread)
                                  == indexed__Traversable_Collection.end(),
                                  "There were not propositions parsed in this descriptive element.");
        
        auto traversable_Collection = indexed__Traversable_Collection[runtime_Thread];

        QUERY_UNRECOVERABLE_ERROR(!traversable_Collection.use_count(),
                                  "There were not propositions parsed in this descriptive element.");
        
        QUERY_UNRECOVERABLE_ERROR(!(*index < (*traversable_Collection).size()),
                                  "We parsed :: "<<(*traversable_Collection).size()<<" elements"<<std::endl
                                  <<"And are asking for element :: "<<*index<<std::endl;);
        
        
        auto proposition = (*traversable_Collection)[*index];
        if(potential_match_via_an_assignment(state_Predicate,
                                             std::tr1::get<0>(proposition),
                                             std::tr1::get<1>(proposition))) return true;
    }
    
    INTERACTIVE_VERBOSER(true, 3101, "No ground instances to match "<<state_Predicate<<std::endl);
    
    return false;
}

bool Formula_Data::statically_unsatisfiable(const Planning::Formula::State_Predicate& state_Predicate) const
{
    return !Formula_Data::statically_unsatisfiable(state_Predicate);
//     auto predicate_Name = state_Predicate.get__name();

//     auto _occurrence_indices = state_propositions__parsed.find(predicate_Name);
//     if(_occurrence_indices == state_propositions__parsed.end()) return true;

//     auto occurrence_indices = _occurrence_indices->second;

//     for(auto index = occurrence_indices.begin()
//             ; index != occurrence_indices.end()
//             ; index++){

//         assert(Planning::Formula::State_Proposition::
//                indexed__Traversable_Collection.find(reinterpret_cast<type_wrapper::runtime_Thread>(this))
//                != Planning::Formula::State_Proposition::
//                indexed__Traversable_Collection.end());
        
//         auto traversable_Collection = Planning::Formula::State_Proposition::
//             indexed__Traversable_Collection[reinterpret_cast<type_wrapper::runtime_Thread>(this)];

//         assert(traversable_Collection.find(index) != traversable_Collection.end());
        
//         auto proposition = traversable_Collection[index];
        
//         if(potential_match_via_an_assignment(state_Predicate, proposition)) {return false;}
//     }
    
//     return true; 
}

const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >&
Formula_Data::get__modified_in_effect__perceptual_functions__parsed() const
{
    return modified_in_effect__perceptual_functions__parsed;
}

const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >&
Formula_Data::get__modified_in_effect__perceptual_ground_functions__parsed() const
{
    return modified_in_effect__perceptual_ground_functions__parsed; 
}

const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >& Formula_Data::get__modified_in_effect__state_functions__parsed() const
{
    return modified_in_effect__state_functions__parsed;
}

const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >& Formula_Data::get__modified_in_effect__state_ground_functions__parsed() const
{
    return modified_in_effect__state_ground_functions__parsed;
}

const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >& Formula_Data::get__state_functions__parsed() const 
{
    return state_functions__parsed;
}

const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >& Formula_Data::get__perceptual_functions__parsed() const
{
    return perceptual_functions__parsed;
}

const std::map<Planning::State_Function_Name, std::set<ID_TYPE> >& Formula_Data::get__state_ground_functions__parsed() const
{
    return state_ground_functions__parsed;
}

const std::map<Planning::Perceptual_Function_Name, std::set<ID_TYPE> >& Formula_Data::get__perceptual_ground_functions__parsed() const
{
    return perceptual_ground_functions__parsed;
}

const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& Formula_Data::get__state_propositions__parsed() const
{
    return state_propositions__parsed;
}

const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& Formula_Data::get__state_predicates__parsed() const
{
    return state_predicates__parsed;
}

const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& Formula_Data::get__perceptual_propositions__parsed() const
{
    return perceptual_propositions__parsed;
}

const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& Formula_Data::get__perceptual_predicates__parsed() const
{
    return perceptual_predicates__parsed; 
}

const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& Formula_Data::get__deleted__state_propositions__parsed() const
{
    return deleted__state_propositions__parsed;
}

const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& Formula_Data::get__deleted__state_predicates__parsed() const
{
    return deleted__state_predicates__parsed;
}

const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& Formula_Data::get__deleted__perceptual_propositions__parsed() const
{
    return deleted__perceptual_propositions__parsed;
}

const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& Formula_Data::get__deleted__perceptual_predicates__parsed() const
{
    return deleted__perceptual_predicates__parsed;
}

const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& Formula_Data::get__added__state_propositions__parsed() const
{
    return added__state_propositions__parsed;
}

const std::map<Planning::Predicate_Name, std::set<ID_TYPE> >& Formula_Data::get__added__state_predicates__parsed() const
{
    return added__state_predicates__parsed;
}

const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& Formula_Data::get__added__perceptual_propositions__parsed() const
{
    return added__perceptual_propositions__parsed;
}

const std::map<Planning::Percept_Name, std::set<ID_TYPE> >& Formula_Data::get__added__perceptual_predicates__parsed() const
{
    return added__perceptual_predicates__parsed;
}

   
bool Formula_Data::is_type__double(const Planning::State_Function_Name&) const
{
    return false;
}

bool Formula_Data::is_type__int(const Planning::State_Function_Name&) const
{
    return false;
}
bool Formula_Data::is_type__double(const Planning::Perceptual_Function_Name&) const
{
    return false;
}
bool Formula_Data::is_type__int(const Planning::Perceptual_Function_Name&) const
{
    return false;
}     

bool Formula_Data::is_type__number(const Planning::State_Function_Name&) const
{
    return false;
}

bool Formula_Data::is_type__number(const Planning::Perceptual_Function_Name&) const
{
    return false;
}     

// Formula_Data::modal_truth
// Formula_Data::statically_unsatisfiable(const Planning::Formula::State_Predicate& state_Predicate,
//                                        const std::map<Variable,  Constants&>& assignment_possibilities) const
// {
//     auto predicate_Name = state_Predicate.get__name();
//     auto arguments = state_Predicate.get__arguments();
    
    
//     auto _occurrence_indices = state_propositions__parsed.find(predicate_Name);
//     if(_occurrence_indices == state_propositions__parsed.end()) return modal_truth::necessarily_true;
    
//     auto occurrence_indices = _occurrence_indices->second;

//     /* Which of the symbol (\member{state_Predicate}) arguments are
//      * variable? -- Result is stored in \local{important_constants}.*/
//     map<uint, Constants> important_constants;
//     for(auto index = 0; index < arguments.size(); index++){
//         auto element = arguments[i];
//         if(element->try_cast<Planning::Variable>()){
//             important_constants[index] = Constants();
//         }
//     }

//     QUERY_UNRECOVERABLE_ERROR(!important_constants.size(),
//                               "None of the argument to predicate symbol :: "<<state_Predicate<<std::endl
//                               <<"Are variable."<<std::endl);
    
    
//     for(auto index = occurrence_indices.begin()
//             ; index != occurrence_indices.end()
//             ; index++){

//         auto query__runtime_thread = reinterpret_cast<type_wrapper::runtime_Thread>(this);
//         auto indexed__Traversable_Collection = Planning::Formula::State_Proposition::indexed__Traversable_Collection;
        
//         auto _traversable_Collection = indexed__Traversable_Collection.find(query__runtime_thread);
//         QUERY_UNRECOVERABLE_ERROR(_traversable_Collectio
//                                   == indexed__Traversable_Collection.end(),
//                                   "Cannot find \class{State_Proposition}s associated with current object.");
        
//         auto traversable_Collection = _traversable_Collectio->second;

//         QUERY_UNRECOVERABLE_ERROR(index >= traversable_Collection.size(),
//                                   "We were expectng a proposition at index :: "<<index<<std::endl
//                                   <<"However we have only parsed :: "<<traversable_Collection.size()
//                                   <<" propositions for this structure."<<std::endl);
        
//         auto proposition = traversable_Collection[index];

        
//         if(potential_match_via_an_assignment(state_Predicate, proposition)){
//             auto propositionx_argument = proposition.get__arguments();
            
//             for(auto important_indices = important_constants.begin()
//                     ; important_indices != important_constants.end()
//                     ; important_indices ++){
//                 important_indices->second.insert(propositionx_argument[important_indices->first]);
//             }            
//         } else {
//             /*The argument fact is necessarily false.*/
//             return modal_truth::possibly_true;
//         }
//     }

//     bool all_possible_instances_included = true;
//     for(auto important_indices = important_constants.begin()
//             ; important_indices != important_constants.end()
//             ; important_indices ++){
        
//         assert(arguments[important_indices->first].test_cast<Variable>());
        
//         auto variable = arguments[important_indices->first].get<Variable>();

//         assert(assignment_possibilities.find(*variable) != assignment_possibilities.end());
        
//         if(*assignment_possibilities.find(*variable) != important_indices->second){
//             all_possible_instances_included = false;
//             break;
//         }
//     }

//     if(all_possible_instances_included) return modal_truth::necessarily_false;

//     return modal_truth::possibly_true;
// }

