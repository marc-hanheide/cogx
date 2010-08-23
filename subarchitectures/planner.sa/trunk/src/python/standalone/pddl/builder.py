import mapltypes as types
import scope, predicates, conditions, effects, actions, domain
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

    def get_arg(self, arg):
        if isinstance(arg, (int, float, long)):
            return predicates.Term(arg)
        if isinstance(arg, (list, tuple)):
            return self(*arg)
        if arg in self.scope:
            return self.scope[arg]
        return arg
        
    def __call__(self, *args):
        args = [self.get_arg(a) for a in args]

        #is the first argument a function?
        if isinstance(args[0], predicates.Function):
            func = args[0]
        elif isinstance(args[0], str):
            func = self.scope.functions.get(args[0], args[1:])
            if not func:
                func = self.scope.functions.get(args[0], args[1:])
        else:
            return args[0]

        if isinstance(func, list):
            func = func[0]
            
        if isinstance(func, predicates.Predicate):
            return predicates.Literal(func, args[1:], self.scope)
        else:
            return predicates.FunctionTerm(func, args[1:])

        assert(False)

    def effect(self, *args):
        args = [self.get_arg(a) for a in args]
        first = self.scope.predicates.get(args[0], args[1:], SCOPE_EFFECT)
        if first:
            return effects.SimpleEffect(first, args[1:], self.scope)
        assert(False)

    def cond(self, *args):
        args = [self.get_arg(a) for a in args]
        first = self.scope.predicates.get(args[0], args[1:], SCOPE_CONDITION)
        if first:
            return conditions.LiteralCondition(first, args[1:], self.scope)
        assert(False)

    def neg(self, arg):
        return self.get_arg(arg).negate()
    def con(self, *args):
        args = [self.get_arg(a) for a in args]
        return conditions.Conjunction(args, self.scope)
    def dis(self, *args):
        args = [self.get_arg(a) for a in args]
        return conditions.Disjunction(args, self.scope)
        
    

# tags = {
#     "and" : lambda b, args: conditions.Conjunction([b(arg) for arg in args])
#     "or" : lambda b, args: conditions.Disjunction([b(arg) for arg in args])
