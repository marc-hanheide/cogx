# Constants for global use

from string import Template


################################################
# Syntactic elements of MAPL and PDDL
################################################

TRUE_STRING = "true"
FALSE_STRING = "false"

K_PREFIX = "k"
DIRECT_K_PREFIX = "kd"

NEGOTIATE_PLAN_OP = "negotiate_plan"
CAN_TALK_TO = "can_talk_to"
COMMITED_TO_PLAN = "commited_to_plan"

SUBGOAL_OP_EXECUTING = """
(:action __achieve_subgoal_$subgoal_name
 :agent (?a - planning_agent)
 :precondition $goal_condition
 :effect (and
	(achieved $subgoal_name))
)
"""

SUBGOAL_OP_REQUESTING = """
(:action wait_for_subgoal_$subgoal_name
 :agent (?a - agent)
:precondition (= ?a $agt_name)
:effect (and
        $goal_condition
        )
)
"""

NEGOTIATE_PLAN_OP_TMPL = """
(:action $NEGOTIATE_PLAN_OP
 :agent (?pa - planning_agent)
 :parameters (?a - agent)
 :precondition (and 
    ($CAN_TALK_TO ?pa ?a))
 :effect (and
    ($COMMITED_TO_PLAN ?a)
))
"""
tmpl = Template(NEGOTIATE_PLAN_OP_TMPL)
NEGOTIATE_PLAN_OP_TMPL = tmpl.safe_substitute(locals())

SPECIAL_OPERATORS = [
#    EQUALS_AXIOM,
#    K_EQUALS_AXIOM,
    NEGOTIATE_PLAN_OP_TMPL
]



################################################
# Conversion from MAPL to PDDL
################################################

MAPL_BASE_ONTOLOGY = """
  agent phys_obj - object
  subgoal feature boolean - object
  planning_agent - agent
""" 

MAPL_BASE_OBJECTS = "true false - boolean"

IS_PLANNING_AGENT_DECL = "is_planning_agent ?_a - agent"

MAPL_BASE_PREDICATES =  [
    IS_PLANNING_AGENT_DECL,
    "equals ?o1 ?o2 - object",
    "achieved ?sg - subgoal",
    "commited_to_plan ?a - agent",
    "can_talk_to ?a1 ?a2 - agent"
    ]

HIDDEN_PREFIX = "__"
PDDL_SENSOR_PREFIX = HIDDEN_PREFIX + "sensor__"
PDDL_NEG_SENSOR_PREFIX = PDDL_SENSOR_PREFIX + "neg__"
ACHIEVE_SG_PREFIX = HIDDEN_PREFIX + "achieve_subgoal_"


################################################
# Produce DOT files
################################################

DOT_TEMPLATE = """
digraph $name {
$setup
// nodes
$node_decl
$ranks
// edges
$edge_decl
}
"""

DOT_SETUP_TMPL = """
// standard options
node [color=black, fontsize=12, style=solid]
edge [color=black, fontsize=10, style=solid] 
"""

