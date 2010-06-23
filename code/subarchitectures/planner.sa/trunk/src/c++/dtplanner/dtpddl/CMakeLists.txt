# Author: Charles Gretton (charles.gretton@gmail.com)
#

include_directories(.)
include_directories(./pegtl-0.30/include)

set(dtpddl_source 
  Command_Line_Arguments.cc
  dtp_pddl.cc
  dtp_pddl_parsing_actions.cc
  dtp_pddl_parsing_actions_domain.cc
  dtp_pddl_parsing_actions_problem.cc
  dtp_pddl_parsing_data.cc
  dtp_pddl_parsing_data_constants.cc
  dtp_pddl_parsing_data_domain__actions.cc
  dtp_pddl_parsing_data_domain.cc
  dtp_pddl_parsing_data_domain__observations.cc
  dtp_pddl_parsing_data_domain__streaming.cc
  dtp_pddl_parsing_data_formula.cc
  dtp_pddl_parsing_data_problem.cc
  dtp_pddl_parsing_data_problem__streaming.cc
  dtp_pddl_parsing_data_types.cc
  dtp_pddl_parsing_interface.cc
  dtp_pddl_parsing_interface_domain.cc
  dtp_pddl_parsing_interface_problem.cc
  global.cc
  planning_action_schema.cc
  planning_derived_predicate.cc
  planning_formula.cc
  planning_observation_schema.cc
  planning_symbols.cc
  stl__typed_thing.cc
  turnstyle.cc)

add_definitions(-std=c++0x)

add_definitions(-pedantic -vg -Wall -Wextra)

add_library(dtpddl ${dtpddl_source})
target_link_libraries(dtpddl pthread)
