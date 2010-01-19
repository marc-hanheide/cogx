import mapltypes
from mapltypes import Parameter, TypedObject, Type, FunctionType
from mapltypes import t_object, t_boolean, t_number
from mapltypes import TRUE, FALSE, UNKNOWN, UNDEFINED

from predicates import Predicate, Function

#predefined types
default_types = [t_object, t_boolean]

#basic predicates
equals = Predicate("=", [Parameter("?o1", t_object), Parameter("?o2", t_object)], builtin=True)

assign = Predicate("assign", [Parameter("?f", FunctionType(t_object)), Parameter("?v", t_object)], builtin=True)
change = Predicate("change", [Parameter("?f", FunctionType(t_object)), Parameter("?v", t_object)], builtin=True)
equal_assign = Predicate("=", [Parameter("?f", FunctionType(t_object)), Parameter("?v", t_object)], builtin=True)

#numeric predicates
num_assign = Predicate("assign", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)
num_change = Predicate("change", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)
num_equal_assign = Predicate("=", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)

scale_up = Predicate("scale-up", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)
scale_down = Predicate("scale-down", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)
increase = Predicate("increase", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)
decrease = Predicate("decrease", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True)

numeric_ops = [num_assign, scale_up, scale_down, increase, decrease]
assignment_ops = [assign, change, num_assign, equal_assign, num_equal_assign]

gt = Predicate(">", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True)
lt = Predicate("<", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True)
eq = Predicate("=", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True)
ge = Predicate(">=", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True)
le = Predicate("<=", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True)

numeric_comparators = [gt, lt, eq, ge, le]

#numeric functions
minus = Function("-", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)
plus = Function("+", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)
mult = Function("*", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)
div = Function("/", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)

neg = Function("-", [Parameter("?n", t_number)], t_number, builtin=True)

numeric_functions = [minus, plus, mult, div, neg]

#default minimization functions
total_time = Function("total-time", [], t_number, builtin=True)
total_cost = Function("total-cost", [], t_number, builtin=True)


