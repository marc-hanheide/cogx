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
        if result:
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
def collect_functions(elem, results=[]):
    if elem.__class__ == predicates.FunctionTerm:
        return sum(results, []) + [elem.function]
    if isinstance(elem, predicates.Literal):
        return sum([t.visit(collect_functions) for t in elem.args], [])
        
