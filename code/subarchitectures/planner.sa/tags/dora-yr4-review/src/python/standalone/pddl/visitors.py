import conditions, effects, predicates

def visit(elem, visitor, default=None):
    """Helper function to avoid all those tedious None-checks"""
    if elem is not None:
        return elem.visit(visitor)
    return default

def copy(f):
    """Decorator that makes a visitor copy all unhandled elements"""
    
    def copy_visitor(elem, results):
        result = f(elem, results)
        if result is not None:
            return result
        if isinstance(elem, effects.ProbabilisticEffect):
            return elem.copy(new_parts = filter(lambda (p,e): bool(e), results))
        return elem.copy(new_parts = filter(None, results))
    return copy_visitor

def replace(f):
    """Decorator that makes a visitor replace the parts of all unhandled elements"""
    
    def replace_visitor(elem, results):
        result = f(elem, results)
        if result is not None:
            return result

        results = filter(None, results)

        if isinstance(elem, (effects.ConjunctiveEffect, conditions.JunctionCondition)):
            elem.parts = results
        elif isinstance(elem, conditions.QuantifiedCondition):
            elem.condition = results[0]
        elif isinstance(elem, (effects.UniversalEffect, effects.ConditionalEffect)):
            elem.effect = results[0]
        elif isinstance(elem, (conditions.PreferenceCondition, conditions.IntermediateCondition)):
            elem.cond = results[0]
        elif isinstance(elem, effects.ProbabilisticEffect):
            elem.effects = filter(lambda (p,e): bool(e), results)
        return elem
    return replace_visitor

def collect(f):
    """Decorator that collects the results from all unhandled elements"""
    
    def collect_visitor(elem, results):
        if not isinstance(elem, effects.ProbabilisticEffect):
            input = sum(results, [])
        else:
            input = results
        result = f(elem, input)
        if result is not None:
            if not isinstance(result, list):
                return [result]
            return result
        
        if isinstance(elem, effects.ProbabilisticEffect):
            results = [e for p,e in results]
        return sum(results, [])
    return collect_visitor


@copy
def flatten(elem, results):
    if isinstance(elem, (conditions.JunctionCondition, effects.ConjunctiveEffect)):
        merge = []
        for el in results:
            if el.__class__ == elem.__class__:
                merge += el.parts
            else:
                merge.append(el)
        return elem.__class__(merge)
    elif isinstance(elem, conditions.QuantifiedCondition):
        if elem.condition.__class__ == elem.__class__:
            new = elem.copy(new_parts = conditions.Truth())
            new.args += elem.condition.args
            new.add(elem.condition.args)
            new.condition = elem.condition.condition
            new.condition.set_scope(new)
            return new
    elif isinstance(elem, effects.UniversalEffect):
        if elem.effect.__class__ == elem.__class__:
            new = elem.copy(new_parts = effects.ConjunctiveEffect([]))
            new.args += elem.effect.args
            new.add(elem.effect.args)
            new.effect = elem.effect.effect
            new.effect.set_scope(new)
            return new
    
def to_list(elem, results):
    """Create lists out of Conjunctions"""
    if isinstance(elem, conditions.Conjunction):
        return sum(results, [])
    if isinstance(elem, effects.ConjunctiveEffect):
        return sum(results, [])
    return [elem]

@collect
def collect_literals(elem, results):
    """Return a list of all literals"""
    if isinstance(elem, predicates.Literal):
        return elem

@collect
def collect_conditions(elem, results):
    """Return a list of all conditions"""
    if isinstance(elem, conditions.LiteralCondition):
        return elem

@collect
def collect_effects(elem, results):
    """Return a list of all effects"""
    if isinstance(elem, effects.SimpleEffect):
        return elem
    
@collect
def collect_functions(elem, results=[]):
    if elem.__class__ == predicates.FunctionTerm:
        return sum(results, []) + [elem.function]
    if isinstance(elem, predicates.Literal):
        return sum([t.visit(collect_functions) for t in elem.args], [])

@collect
def collect_all_functions(elem, results=[]):
    if elem.__class__ == predicates.FunctionTerm:
        return sum(results, []) + [elem.function]
    if isinstance(elem, predicates.Literal):
        return sum([t.visit(collect_functions) for t in elem.args], [elem.predicate])

@collect
def collect_non_builtins(elem, results=[]):
    if elem.__class__ == predicates.FunctionTerm:
        if not elem.function.builtin:
            return sum(results, []) + [elem.function]
    if isinstance(elem, predicates.Literal):
        init = []
        if not elem.predicate.builtin:
            init = [elem.predicate]
        return sum([t.visit(collect_functions) for t in elem.args], init)
    
@collect
def collect_constants(cond, results=[]):
    if isinstance(cond, predicates.Term):
        if isinstance(cond, predicates.VariableTerm):
            return []
        if isinstance(cond, predicates.ConstantTerm):
            return cond.object
    elif isinstance(cond, predicates.Literal):
        return sum([t.visit(collect_constants) for t in cond.args], [])

@collect
def collect_free_vars(elem, results=[]):
    if isinstance(elem, predicates.Term):
        if isinstance(elem, predicates.ConstantTerm):
            return []
        if isinstance(elem, predicates.VariableTerm):
            return elem.object
    elif isinstance(elem, predicates.Literal):
        return sum([t.visit(collect_free_vars) for t in elem.args], [])
    elif isinstance(elem, (conditions.QuantifiedCondition, effects.UniversalEffect)):
        vars = results[0]
        return [p for p in vars if p not in elem.args]
    elif isinstance(elem, effects.ConditionalEffect):
        return results + elem.condition.visit(collect_free_vars)
