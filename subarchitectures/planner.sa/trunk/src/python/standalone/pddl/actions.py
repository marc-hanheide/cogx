#! /usr/bin/env python
# -*- coding: latin-1 -*-
from collections import defaultdict

from parser import ParseError, UnexpectedTokenError
import predicates, conditions, effects, builtin, visitors
from scope import Scope

class Action(Scope):
    """This class represents a PDDl action."""
    
    def __init__(self, name, args, precondition, effect, domain, replan=None):
        """Create a new PDDL action.

        Arguments:
        name -- name of the action
        args -- List of Parameters for this action
        precondition -- a Condition object
        effect -- an Effect object
        domain -- the Domain this action belongs to.
        replan -- Condition object for the replan condition (defaults to None)"""
        
        assert effect != []
        Scope.__init__(self, args, domain)
        self.name = name
        self.args = args

        self.precondition = precondition
        self.replan = replan
        self.effect = effect
        self.totalCostFound = False
        self.totalCostTerm = None

        if self.precondition:
            self.precondition.set_scope(self)
        if self.replan:
            self.replan.set_scope(self)
        if self.effect:
            self.effect.set_scope(self)

    def instantiate(self, mapping, parent=None):
        """Instantiate the Parameters of this action.

        Arguments:
        mapping -- either a dictionary from Parameters to TypedObjects
        or a list of TypedObjects. In the latetr case, the list is
        assumed to be in the order of the Action's Parameters."""
        if not isinstance(mapping, dict):
            mapping = dict((param.name, c) for (param, c) in zip(self.args, mapping))
        Scope.instantiate(self, mapping, parent)

    def to_pddl(self):
        pass
        #str = ["(:action %s" % self.name]
        #indent = len("(:action ")

    def get_inst_func(self, st, condition=None):
        import itertools
        import state
        from predicates import FunctionTerm, FunctionVariableTerm, VariableTerm
        
        def args_visitor(term, results):
            if isinstance(term, FunctionTerm):
                return sum(results, [])
            return [term]

        self.cond_by_arg = defaultdict(set)
        self.free_args = {}
        
        def subcond_visitor(cond, result):
            for arg in cond.free():
                self.cond_by_arg[arg].add(cond)
            if isinstance(cond, conditions.LiteralCondition):
                self.free_args[cond] = cond.free()

        if not condition:
            condition = self.precondition
                
        visitors.visit(condition, subcond_visitor)

        prev_mapping = {}
        checked = set()
            
        def inst_func(mapping, args):
            next_candidates = []
            if checked:
                for k,v in prev_mapping.iteritems():
                    if mapping.get(k, None) != v:
                        checked.difference_update(self.cond_by_arg[k])
            prev_mapping.update(mapping)
            
            def instantianteAndCheck(cond, combinations, func):
                for c in combinations:
                    cond.instantiate(dict(zip(cond.args, c)), st.problem)
                    result = func()
                    cond.uninstantiate()
                    yield result

            #print [a.name for a in mapping.iterkeys()]
            forced = []
            def check(cond):
                def is_instantiated_function(term):
                    return type(term) == FunctionTerm or (type(term) == FunctionVariableTerm and term.is_instantiated())
                
                if not cond or cond in checked:
                    return True
                if isinstance(cond, conditions.LiteralCondition):
                    if cond.predicate == builtin.equals and is_instantiated_function(cond.args[0]) and isinstance(cond.args[1], VariableTerm):
                        v = cond.args[-1]
                        if all(a.is_instantiated() for a in cond.args[0].args if isinstance(a, VariableTerm)) and  isinstance(v, VariableTerm) and not v.is_instantiated():
                            svar = state.StateVariable.from_literal(cond, st)
                            forced.append((v.object, st[svar], cond))
                    
                    if all(a.is_instantiated() for a in self.free_args[cond]):
                        fact = state.Fact.from_literal(cond, st)
                        exst = st.get_extended_state([fact.svar])
                        #TODO: handle all possible conditions
                        if exst[fact.svar] != fact.value:
                            return False
                        checked.add(cond)
                        return True
                    else:
                        next_candidates.append([a for a in self.free_args[cond] if not a.is_instantiated()])
                elif isinstance(cond, conditions.Truth):
                    return True
                elif isinstance(cond, conditions.Falsity):
                    return False
                elif isinstance(cond, conditions.Conjunction):
                    results = [check(c) for c in cond.parts]
                    if any(c == False for c in results):
                        return False
                    if all(c == True for c in results):
                        checked.add(cond)
                        return True
                elif isinstance(cond, conditions.Disjunction):
                    results = [check(c) for c in cond.parts]
                    if any(c == True for c in results):
                        checked.add(cond)
                        return True
                    if all(c == False for c in results):
                        return False
                elif isinstance(cond, conditions.QuantifiedCondition):
                    combinations = itertools.product(*map(lambda a: list(st.problem.get_all_objects(a.type)), cond.args))
                    results = list(instantianteAndCheck(cond, combinations, lambda: check(cond.condition)))
                    if isinstance(cond, conditions.Conjunction):
                        if any(c == False for c in results):
                            return False
                        if all(c == True for c in results):
                            checked.add(cond)
                            return True
                    elif isinstance(cond, conditions.ExistentialCondition):
                        if any(c == True for c in results):
                            checked.add(cond)
                            return True
                        if all(c == False for c in results):
                            return False
                else:
                    assert False
                return None

            result = check(condition)
            if result == True:
                checked.add(condition)
                #print self.name, "accept:", [a.get_instance().name for a in  args]
                return True, None
            elif result == False:
                #print self.name, "reject:", [a.get_instance().name for a in  args]
                return None, None
            
            if forced:
                svar, val, lit = forced[0]
                checked.add(lit)
                #print "Forced %s = %s" % (str(svar), val.name)
                return (svar, val)
            if next_candidates:
                next_candidates = sorted(next_candidates, key=lambda l: len(l))
                #print "Next:", next_candidates[0][0]
                return next_candidates[0][0], None
            #print self, [a.get_instance().name for a in  args]
            return True, None
        return inst_func

    def get_effects(self, term):
        @visitors.collect
        def visitor(eff, parts):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate in builtin.numeric_ops + builtin.assignment_ops:
                    if eff.args[0] == term:
                        return eff.args[1]
        return visitors.visit(self.effect, visitor, [])
        
    
    def get_total_cost(self):
        tct = predicates.Term(builtin.total_cost,[])

        res = self.get_effects(tct)
        return res[0] if res else None

    def set_total_cost(self, new_total_cost):
        self.totalCostFound = False
        self.totalCostTerm = None
        tct = predicates.Term(builtin.total_cost,[])

        @visitors.replace
        def visitor(eff, parts):
            if isinstance(eff, effects.SimpleEffect):
                if eff.predicate == builtin.increase:
                    if eff.args[0] == tct:
                        self.totalCostFound = True
                        if not new_total_cost:
                            return False
                        new_cost_eff = effects.SimpleEffect(builtin.increase,[tct,new_total_cost], eff.scope)
                        return new_cost_eff

        self.effect = self.effect.visit(visitor)

        if not self.totalCostFound and new_total_cost is not None:
            new_cost_eff = effects.SimpleEffect(builtin.increase,[tct,new_total_cost], self.effect.scope)
            if isinstance(self.effect, effects.ConjunctiveEffect):
                self.effect.parts.append(new_cost_eff)
            else:
                effs = [self.effect,new_cost_eff]
                new_eff = effects.ConjunctiveEffect(effs,self.effect.scope)
                self.effect = new_eff


    def copy(self, newdomain=None):
        """Create a deep copy of this Action.

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent
            
        a = Action(self.name, [], None, None, newdomain)
        a.args = a.copy_args(self.args)
        
        if self.precondition:
            a.precondition = self.precondition.copy(a)
        if self.replan:
            a.replan = self.replan.copy(a)
        if self.effect:
            a.effect = self.effect.copy(a)

        return a

    def copy_skeleton(self, newdomain=None):
        """Create a copy of this action's skeleton (name, arguments
        but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""
        if not newdomain:
            newdomain = self.parent

        a = Action(self.name, [], None, None, newdomain)
        a.args = a.copy_args(self.args)
        return a

    def _extend_precond_or_effect(self, old, new_part, JunctionType):
        if isinstance(old, JunctionType):
            old.parts.append(new_part)
            return old
        elif old:
            return JunctionType([old, new_part], scope=old.get_scope())
        else:
            return JunctionType([new_part], scope=None)

    def extend_precondition(self, new_cond):
        self.precondition = self._extend_precond_or_effect(self.precondition, new_cond, conditions.Conjunction)
    
    def extend_effect(self, new_eff):
        self.effect = self._extend_precond_or_effect(self.effect, new_eff, effects.ConjunctiveEffect)
    
    @staticmethod
    def parse(it, scope):
        it.get(":action")
        name = it.get().token.string
        next = it.get()

        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
        
        action = Action(name, params, None, None, scope)

        try:
            while True:
                if next.token.string == ":precondition":
                    if action.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    action.precondition = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action)
                else:
                    raise UnexpectedTokenError(next.token)
                    
                next = it.next()

        except StopIteration:
            pass
            
        return action

