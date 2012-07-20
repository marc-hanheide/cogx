import builtin, predicates

pddl_module = True

virtual_cost = predicates.Function("virtual-cost", [], builtin.t_number, builtin=True)

default_functions = [virtual_cost]

