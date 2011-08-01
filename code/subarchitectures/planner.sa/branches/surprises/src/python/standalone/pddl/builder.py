import mapltypes as types
import scope, predicates, conditions, effects, state
from scope import SCOPE_CONDITION, SCOPE_EFFECT, SCOPE_ALL, SCOPE_INIT

# class build(object):
#     def __init__(self, elem):
#         self.elem = elem
        
#     def __enter__(self):
#         return Builder(elem)

#     def __exit__(self, type, value, traceback):
#         pass
    
class Builder(object):
    def __init__(self, elem):
        self.elem = elem
        if isinstance(elem, scope.Scope):
            self.scope = elem
        else:
            self.scope = elem.scope

    def get_arg(self, arg, **kwargs):
        if isinstance(arg, (int, float, long)):
            return predicates.Term(arg)
        if isinstance(arg, (list, tuple)):
            return self(*arg, **kwargs)
        if isinstance(arg, predicates.Term):
            return arg.copy(self.scope)
        if isinstance(arg, (types.TypedObject, str)) and arg in self.scope:
            return self.scope[arg]
        return arg
        
    def __call__(self, *args, **kwargs):
        function_scope = kwargs.get("function_scope", SCOPE_ALL)

        if args[0] == "when":
            cond = self(*args[1], function_scope=SCOPE_CONDITION)
            eff = self(*args[2], function_scope=SCOPE_EFFECT)
            return effects.ConditionalEffect(cond, eff, self.scope)

        args = [self.get_arg(a, **kwargs) for a in args]

        if args[0] == "not":
            return args[1].negate()
        if args[0] == "and" and function_scope & SCOPE_CONDITION:
            return conditions.Conjunction(args[1:], self.scope)
        if args[0] == "and" and function_scope & SCOPE_EFFECT:
            return effects.ConjunctiveEffect(args[1:], self.scope)
        
        #is the first argument a function?
        if isinstance(args[0], predicates.Function):
            func = args[0]
        elif isinstance(args[0], str):
            func = self.scope.functions.get(args[0], args[1:], function_scope)
            if not func:
                func = self.scope.predicates.get(args[0], args[1:], function_scope)
        else:
            return args[0]

        if isinstance(func, list):
            func = func[0]
            
        if isinstance(func, predicates.Predicate):
            if function_scope & SCOPE_CONDITION and not function_scope & SCOPE_EFFECT:
                return conditions.LiteralCondition(func, args[1:], self.scope)
            elif not function_scope & SCOPE_CONDITION and function_scope & SCOPE_EFFECT:
                return effects.SimpleEffect(func, args[1:], self.scope)
            return predicates.Literal(func, args[1:], self.scope)
        else:
            return predicates.FunctionTerm(func, args[1:], self.scope)

        assert(False)

    def effect(self, *args):
        return self(*args, function_scope=SCOPE_EFFECT)

    def timed_effect(self, *args):
        import durative
        lit = self(*args[1:], function_scope=SCOPE_EFFECT)
        return durative.TimedEffect(lit.predicate, lit.args, args[0], self.scope, lit.negated)

    def cond(self, *args):
        return self(*args, function_scope=SCOPE_CONDITION)

    def init(self, *args):
        return self(*args, function_scope=SCOPE_INIT)
    
    def neg(self, arg):
        return self.get_arg(arg).negate()
    def con(self, *args):
        args = [self.get_arg(a) for a in args]
        return conditions.Conjunction(args, self.scope)
    def dis(self, *args):
        args = [self.get_arg(a) for a in args]
        return conditions.Disjunction(args, self.scope)

    def svar(self, *args):
        args = [self.get_arg(a) for a in args]
        #is the first argument a function?
        if isinstance(args[0], predicates.Function):
            func = args[0]
        elif isinstance(args[0], str):
            func = self.scope.functions.get(args[0], args[1:])
            if not func:
                func = self.scope.predicates.get(args[0], args[1:])
        return state.StateVariable(func, args[1:])
    

# tags = {
#     "and" : lambda b, args: conditions.Conjunction([b(arg) for arg in args])
#     "or" : lambda b, args: conditions.Disjunction([b(arg) for arg in args])
