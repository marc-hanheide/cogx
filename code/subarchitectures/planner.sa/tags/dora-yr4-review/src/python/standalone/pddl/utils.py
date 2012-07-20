from itertools import chain
import builtin, predicates, visitors
import mapltypes as types

def is_functional(lit):
    return lit.predicate in (builtin.equals, builtin.eq, builtin.assign, builtin.num_assign, builtin.equal_assign, builtin.num_equal_assign)

def get_function(lit):
    if is_functional(lit):
        if isinstance(lit.args[0], predicates.FunctionTerm):
            return lit.args[0].function
    return lit.predicate

def get_literal_elements(lit):
    function = None
    litargs = []
    modality = None
    modal_args = []
    value = None
    negated = False
    
    if is_functional(lit):
        function = lit.args[0].function
        litargs = lit.args[0].args
        value = lit.args[1]
        modal_args = []
        negated = lit.negated
    else:
        for arg, parg in zip(lit.args, lit.predicate.args):
            if isinstance(parg.type, predicates.FunctionType):
                if function is None:
                    function = arg.function
                    litargs = arg.args
                else:
                    assert False, "A modal svar can only contain one function"
            else:
                modal_args.append(arg)

        if not function:
            function = lit.predicate
            litargs = lit.args
            modal_args = []
        else:
            modality = lit.predicate

        value = predicates.ConstantTerm(builtin.TRUE if not lit.negated else builtin.FALSE)

    return function, litargs, modality, modal_args, value, negated

def to_object(thing):
    if isinstance(thing, predicates.Term):
        return thing.object
    return thing

def match_arguments(a1, a2, mapping):
    a1 = to_object(a1)
    a2 = to_object(a2)
    if not isinstance(a1, types.Parameter):
        return a1 == a2
    if not a2.is_instance_of(a1.type):
        return False
    return mapping.get(a1, a2) == a2

def fact_matches_literal(fact, lit, mapping = {}):
    func, args, mod, modargs, value, neg = get_literal_elements(lit)
    if fact.svar.function != func or fact.svar.modality != mod:
        return None
    mapping = dict(mapping)
    for a1, a2 in chain(zip(args, fact.svar.args), zip(modargs, fact.svar.modal_args), [(value, fact.value)]):
        if match_arguments(a1, a2, mapping):
            mapping[to_object(a1)] = to_object(a2)
        else:
            return None
    return mapping

def get_unbound_parameters(lit):
    @visitors.collect
    def visitor(elem, results):
        if isinstance(elem, predicates.VariableTerm):
            if not elem.is_instantiated():
                return elem
    return lit.v

# def incremental_matching(literals, mapping):
#     def unbounded(l):
#         func, args, mod, modargs, value, neg = get_literal_elements(l)
#         for a in chain(args, modargs):
            
        
#     ldict = defaultdict(list)
#     for l in literals:
#         for
        
        

def can_satisfy(lit1, lit2):
    (func1, args1, mod1, modargs1, val1, neg1) = get_literal_elements(lit1)
    (func2, args2, mod2, modargs2, val2, neg2) = get_literal_elements(lit2)
    if (func1 != func2 or mod1 != mod2):
        return False

    def check_compat(a1, a2):
        if isinstance(a1, predicates.ConstantTerm) and isinstance(a2, predicates.ConstantTerm):
            return a1 == a2
        return a1.equal_or_subtype_of(a2) or a2.equal_or_subtype_of(a1)

    if any(not check_compat(a, a2)  for a,a2 in zip(args1, args2)):
        return False
    if any(not check_compat(a, a2) for a,a2 in zip(modargs1, modargs2)):
        return False

    
