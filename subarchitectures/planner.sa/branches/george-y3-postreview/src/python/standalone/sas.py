import time
from itertools import chain, product, izip
from collections import defaultdict


import pddl
from pddl import utils, builtin, state, effects, dtpddl, mapl, visitors


def is_grounded(arg):
    if not isinstance(arg, pddl.types.Parameter):
        return True
    if not arg.is_instantiated() or isinstance(arg.get_instance(), pddl.types.Parameter):
        return False
    return True

def get_instance(a):
    if isinstance(a, pddl.types.Parameter) and a.is_instantiated():
        return a.get_instance()
    return a

def instantiate_args(args):
    result = []
    for arg in args:
        if arg.__class__ in (pddl.VariableTerm, pddl.types.Parameter) and arg.is_instantiated():
            result.append(arg.get_instance())
        elif isinstance(arg, (pddl.ConstantTerm, pddl.VariableTerm)):
            result.append(arg.object)
        elif isinstance(arg, pddl.types.TypedObject):
            result.append(arg)
        else:
            raise Exception("couldn't create state variable, %s is a function term and no state was supplied." % str(arg))
    return result

def match_facts(f1, f2, mapping, match_fn=utils.match_arguments):
    if isinstance(f1, LiftedFact):
        return f1.match(f2, mapping, match_fn)
    elif f1 == f2:
        return mapping.copy()
    return None
        
def match_svars(s1, s2, mapping, match_fn=utils.match_arguments):
    if isinstance(s1, LiftedStateVariable):
        return s1.match(s2, mapping, match_fn)
    elif s1 == s2:
        return mapping.copy()
    return None

def match_factlists(f1, f2, mapping, match_fn=utils.match_arguments):
    def match(list1, list2, mapping):
        if len(list1) != len(list2):
            return None
        for i, f1 in enumerate(list1):
            for j, f2 in enumerate(list2):
                m2 = match_facts(f1, f2, mapping, match_fn)
                if m2 is not None:
                    m2 = match(list1[:i] + list1[i+1:], list2[:j] + list2[j+1:], m2)
                    if m2 is not None:
                        return m2
            return None
        return mapping
    return match(f1, f2, mapping)

class LiftedStateVariable(state.StateVariable):
    def __eq__(self, other):
        return isinstance(other, state.StateVariable) and self.hash == hash(other)

    def is_instantiated(self):
        return all(is_grounded(a) for a in self.all_args())

    def copy_instance(self):
        return LiftedStateVariable(self.function, map(get_instance, self.args), self.modality, map(get_instance, self.modal_args))

    def copy_grounded(self):
        assert self.is_instantiated()
        return state.StateVariable(self.function, map(get_instance, self.args), self.modality, map(get_instance, self.modal_args))

    def match(self, other, mapping, match_fn=utils.match_arguments):
        m2 = mapping.copy()
        if self.function != other.function or self.modality != other.modality:
            return None
        for a1, a2 in chain(izip(self.all_args(), other.all_args())):
            if not match_fn(a1, a2, m2):
                return None
            if isinstance(a1, pddl.Parameter) and a1 not in m2:
                m2[a1] = a2
        return m2
    
    def __eq__(self, other):
        return isinstance(other, state.StateVariable) and hash(self) == hash(other)
    
    def __hash__(self):
        return hash((self.function, self.modality) + tuple(map(get_instance, self.args) + map(get_instance, self.modal_args)))

    @staticmethod
    def from_literal(literal, _state=None):
        function, litargs, modality, modal_args, _ = state.StateVariable.svar_args_from_literal(literal)

        args = instantiate_args(litargs)
        if modality is not None:
            modal_args = instantiate_args(modal_args)
            return LiftedStateVariable(function, args, literal.predicate, modal_args)
            
        return LiftedStateVariable(function, args)

    def __str__(self):
        s = "%s(%s)" % (self.function.name, " ".join(get_instance(a).name for a in self.args))
        if self.modality:
            s = "%s(%s, %s)" % (self.modality.name, s, " ".join(get_instance(a).name for a in self.modal_args))
        return s
    
    @staticmethod
    def from_svar(svar):
        return state.StateVariable(svar.function, svar.args, svar.modality, svar.modal_args)

class LiftedFact(state.Fact):
    def __new__(_class, svar, value, negated=False):
        t = tuple.__new__(_class, (svar, value))
        t._negated = negated
        return t

    def negated(self):
        return self._negated

    def match(self, other, mapping, match_fn=utils.match_arguments):
        if self.negated() != other.negated():
            return None
        m2 = self.svar.match(other.svar, mapping, match_fn)
        if m2 is None or not match_fn(self.value, other.value, m2):
            return None
        if isinstance(self.value, pddl.Parameter) and self.value not in m2:
            m2[self.value] = other.value
        return m2
    
    def __hash__(self):
        return hash((self.svar, self.value, self.negated()))

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.svar == other.svar and self.value == other.value and self.negated() == other.negated()
    def __str__(self):
        return "%s %s %s" % (str(self.svar), "!=" if self.negated() else "=",  str(get_instance(self.value).name))
        
    def is_instantiated(self):
        if isinstance(self.svar, LiftedStateVariable) and self.svar.is_instantiated():
            return False
        return isinstance(self.value, pddl.types.Parameter) or self.value.is_instantiated()

    def copy_instance(self):
        val = get_instance(self.value)
        if self.svar.is_instantiated() and is_grounded(self.value):
            if not self.negated():
                return state.Fact(self.svar.copy_instance(), val)
            else:
                return state.NegatedFact(self.svar.copy_instance(), val)
        return LiftedFact(self.svar.copy_instance(), val, self.negated())

    @staticmethod
    def from_literal(literal, state=None):
        value = None
        if utils.is_functional(literal):
            value = instantiate_args(literal.args[-1:])[0]
            return LiftedFact(LiftedStateVariable.from_literal(literal, state), value, literal.negated)
        else:
            if literal.negated:
                value = pddl.FALSE
            else:
                value = pddl.TRUE

        return LiftedFact(LiftedStateVariable.from_literal(literal, state), value)



def extract_exists(condition):
    def extract_exists(cond, results):
        if results:
            new_args, results = zip(*results)
            # print new_args, results
        else:
            new_args = []

        # print cond.pddl_str(), results
        new_args = sum(new_args, [])
        if isinstance(cond, pddl.ExistentialCondition):
            return new_args + cond.args, results[0]
        elif isinstance(cond, pddl.UniversalCondition):
            return [], cond.copy()
        
        return new_args, cond.copy(new_parts = filter(None, results))

    if not condition:
        return [], condition
    
    new_args, cond = visitors.visit(condition, extract_exists)
    if not new_args:
        return [], condition
    return new_args, cond
    
def split_disjunction(condition):
    def get_disjunctions(cond, result):
        # print "---",cond.pddl_str()
        if not result:
            return [cond.copy()]
        
        if isinstance(cond, pddl.Disjunction):
            result = sum(result, [])
            # if not any(set(p.visit(pddl.visitors.collect_all_functions)) & nonstatic for p in result):
            #     return [cond]
            # result.append(cond.parts)
            # print "dis:", result
            return result
        
        # print "nondis",result
        new_results = []
        for new_parts in product(*result):
            # print new_parts
            new_results.append(cond.copy(new_parts=new_parts))
        return new_results
    # print action.precondition.pddl_str() if action.precondition else "--"
    if not condition:
        return []

    disjunctive_parts = pddl.visitors.visit(condition, get_disjunctions, [])
    return disjunctive_parts
        
    # if len(disjunctive_parts) > 1:
    #     eff_args = action.effect.free() if action.effect else set()
    #     result = []
    #     for i, dis in enumerate(disjunctive_parts):
    #         # print dis.pddl_str()
    #         new_args = dis.free() | eff_args
    #         # print map(str, new_args)
    #         da = pddl.Action("%s_%d" % (action.name, i) , [], None, None, domain)
    #         da.args = da.copy_args([a for a in action.args if a in new_args])
    #         da.precondition = dis
    #         da.precondition.set_scope(da)
    #         da.effect = action.effect.copy(new_scope=da)
    #         result.append(da)
    #         # print da.name, dis.pddl_str()
    #     return result
    # return [action]

    

class ActionWrapper(object):
    def __init__(self):
        self.source = None
        self.conditions = []
        self.effects = []
        self.args = []

    @staticmethod
    def create(action):
        if isinstance(action, pddl.actions.Action):
            return PDDLActionWrapper.from_action(action)
        elif isinstance(action, pddl.axioms.Axiom):
            return AxiomWrapper.from_axiom(action)
        elif isinstance(action, dtpddl.DTRule):
            return [DTRuleWrapper(action)]
        else:
            assert False

    def instantiate(self, mapping, parent=None):
        self.action.instantiate(mapping, parent)

    def uninstantiate(self):
        self.action.uninstantiate()
        

class PDDLActionWrapper(ActionWrapper):
    def __init__(self, action, condition = None):
        self.source = action
        if condition is None:
            condition = action.precondition
            
        self.action, condition = self.make_action(action, condition)
        self.args = self.action.args
        
        def make_facts(literals):
            for lit in literals:
                if lit.predicate in pddl.numeric_ops:
                    continue
                yield LiftedFact.from_literal(lit)

        #TODO: support conditional effects
        self.conditions = list(make_facts(visitors.visit(condition, visitors.collect_conditions, default=[])))
        # self.conditions += list(make_facts(visitors.visit(action.effect, visitors.collect_conditions, default=[])))
        self.effects = list(make_facts(visitors.visit(action.effect, visitors.collect_effects, default=[])))

    def make_action(self, action, condition):
        new_args, cond = extract_exists(action.precondition)
        if not new_args:
            return action, condition
        a2 = action.copy()
        a2.args += a2.copy_args(new_args)
        a2.precondition = cond
        a2.precondition.set_scope(a2)
        return a2, cond

    @staticmethod
    def from_action(action):
        for cond in split_disjunction(action.precondition):
            yield PDDLActionWrapper(action, cond)

    def __str__(self):
        return self.action.name

class DTRuleWrapper(ActionWrapper):
    def __init__(self, rule):
        self.source = rule
        self.action = rule
        self.args = rule.args
        self.conditions = [LiftedFact.from_literal(l) for l in rule.conditions]
        self.effects = []
        for _, vals in rule.values:
            for (f, args), v in zip(rule.variables, vals):
                svar = LiftedStateVariable(f, [a.object for a in args])
                self.effects.append(LiftedFact(svar, v.object))

    def __str__(self):
        return str(self.action)

class AxiomWrapper(ActionWrapper):
    def __init__(self, axiom, condition=None):
        self.source = axiom
        if conditions is None:
            condition = axiom.condition

        self.action, condition = self.make_action(axiom, condition)
        self.args = self.action.args

        def make_facts(literals):
            for lit in literals:
                if lit.predicate in pddl.numeric_ops:
                    continue
                yield LiftedFact.from_literal(lit)

        self.conditions = list(make_facts(visitors.visit(condition, visitors.collect_conditions, default=[])))
        self.effects = [LiftedFact.from_literal(effects.SimpleEffect(axiom.predicate, self.action.args, self.action))]

    def make_action(self, axiom, condition):
        new_args, cond = extract_exists(condition)
        if not new_args:
            return axiom, condition
        action = pddl.Action("axiom_%s" % axiom.predicate.name, [], None, None, axiom.parent)
        action.args = action.copy_args(axiom.args) 
        action.effect = effects.SimpleEffect(axiom.predicate, action.args, action)
        action.args += action.copy_args(new_args)
        action.precondition = cond
        action.precondition.set_scope(action)
        return action, cond
            
    @staticmethod
    def from_axiom(axiom):
        for cond in split_disjunction(axiom.condition):
            yield AxiomWrapper(action, cond)
                
    def __str__(self):
        return str(self.action.name)
    
    
