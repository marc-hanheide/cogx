from mapltypes import Parameter, TypedObject, Type, FunctionType, ProxyType
from mapltypes import t_object, t_boolean, t_number, t_any
from mapltypes import TRUE, FALSE, UNKNOWN, UNDEFINED

from predicates import Predicate, Function
from scope import SCOPE_CONDITION, SCOPE_EFFECT, SCOPE_ALL, SCOPE_INIT

#predefined types
default_types = [t_object, t_boolean, t_any]

#basic predicates
equals = Predicate("=", [Parameter("?o1", t_object), Parameter("?o2", t_object)], builtin=True, function_scope=SCOPE_CONDITION)

p = Parameter("?f", FunctionType(t_object))
assign = Predicate("assign", [p, Parameter("?v", ProxyType(p))], builtin=True, function_scope=SCOPE_EFFECT)
equal_assign = Predicate("=", [Parameter("?f", FunctionType(t_object)), Parameter("?v", t_object)], builtin=True, function_scope=SCOPE_INIT)

p = Parameter("?f", FunctionType(t_object))
change = Predicate("change", [p, Parameter("?v", ProxyType(p))], builtin=True, function_scope=SCOPE_EFFECT)
num_change = Predicate("change", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_EFFECT)

#numeric predicates
num_assign = Predicate("assign", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_EFFECT)
num_equal_assign = Predicate("=", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_INIT)

scale_up = Predicate("scale-up", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_EFFECT)
scale_down = Predicate("scale-down", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_EFFECT)
increase = Predicate("increase", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_EFFECT)
decrease = Predicate("decrease", [Parameter("?f", FunctionType(t_number)), Parameter("?v", t_number)], builtin=True, function_scope=SCOPE_EFFECT)

numeric_ops = [num_assign, scale_up, scale_down, increase, decrease]
assignment_ops = [assign, num_assign, equal_assign, num_equal_assign, change, num_change]

gt = Predicate(">", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True, function_scope=SCOPE_CONDITION)
lt = Predicate("<", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True, function_scope=SCOPE_CONDITION)
eq = Predicate("=", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True, function_scope=SCOPE_CONDITION)
ge = Predicate(">=", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True, function_scope=SCOPE_CONDITION)
le = Predicate("<=", [Parameter("?n1", t_number), Parameter("?n2", t_number)], builtin=True, function_scope=SCOPE_CONDITION)

numeric_comparators = [gt, lt, eq, ge, le]

#numeric functions
minus = Function("-", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)
plus = Function("+", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)
mult = Function("*", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)
div = Function("/", [Parameter("?n1", t_number), Parameter("?n2", t_number)], t_number, builtin=True)

neg = Function("-", [Parameter("?n", t_number)], t_number, builtin=True)

numeric_functions = [minus, plus, mult, div, neg]

#default minimization functions
total_cost = Function("total-cost", [], t_number, builtin=True)


