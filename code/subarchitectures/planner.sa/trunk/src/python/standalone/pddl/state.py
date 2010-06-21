#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools, time, random
from itertools import imap, ifilter
from collections import defaultdict

import mapltypes as types
import conditions, problem, effects, durative
from builtin import *
from predicates import *
from mapltypes import TypedNumber

def product(*iterables):
    """Returns an iterator with the cartesian product of the lists
    provided as argument.

    Arguments:
    *iterables -- list of lists
    """
    if not iterables:
        yield tuple()
        return
    for el in iterables[0]:
        if len(iterables) > 1:
            for prod in product(*iterables[1:]):
                yield (el,)+prod
        else:
            yield (el,)
                       

def instantiate_args(args, state=None):
    """Given a list of Terms and a state, try to instantiate these
    terms to ground objects as follows:

    A ConstantTerm will be copied.
    
    A VariableTerm will be replaced by a ConstantTerm of its
    intantiated object. Uninstantiated VariableTerms will cause an
    exception.

    A FunctionTerm or FunctionVariableTerm will be evaluated and
    looked up in the state. A ConstantTerm of that functions's value
    in the state will be returned. If no state is supplied, an
    exception will be raised.

    Arguments:
    args -- list of Term objects
    state -- State object that is used to look up FunctionTerms
    """
    if state:
        return [state.evaluate_term(arg) for arg in args]
    
    result = []
    for arg in args:
        if arg.__class__ == VariableTerm:
            assert arg.is_instantiated(), "%s is not instantiated" % arg.pddl_str()
            result.append(arg.get_instance())
        elif isinstance(arg, ConstantTerm):
            result.append(arg.object)
        else:
            raise Exception("couldn't create state variable, %s is a function term and no state was supplied." % str(arg))
    return result

class StateVariable(object):
    """This class represents a state variable in mapl.

    In its basic variant it is defined by a function and a list of
    (ground) arguments. Additionally a state variable may have a
    modality and a list of arguments specific to that modality."""
    
    def __init__(self, function, args, modality=None, modal_args=[]):
        """Return a new StateVariable.

        Arguments:
        function -- Function this StateVariable represents.
        args -- List of TypedObjects (not Parameters).
        modality -- A Predicate that represents this StateVariable's modality.
        modal_args -- List of TypedObjects (not Parameters)."""
        
        self.function = function
        assert len(function.args) == len(args)
        for a, fa in zip(args, function.args):
            assert a.is_instance_of(fa.type), "%s not of type %s" % (str(a), str(fa))
        self.args = args

        self.modality = modality
        self.modal_args = modal_args
        self.hash = hash((self.function,self.modality)+ tuple(self.args)+tuple(self.modal_args))

    def get_type(self):
        """Return the type of this state variable. If a modality is
        defined, the result will be the type of the modality,
        otherwise the type of the function."""
        if self.modality:
            return self.modality.type
        else:
            return self.function.type

    def get_predicate(self):
        """Return the Predicate of this StateVariable. This will be
        the modality if it exists, the function if it is a Predicate,
        ot None otherwise."""
        if self.modality:
            return self.modality
        if isinstance(self.function, Predicate):
            return self.function
        return None

    def as_modality(self, modality, modal_args=[]):
        """Return a copy of this StateVariable with a modified modality.

        Arguments:
        modality -- The Predicate defining the new modality
        modal_args -- List of TypedObjects depending on the modality
        """
        return StateVariable(self.function, self.args, modality, modal_args)

    def nonmodal(self):
        """Return a copy of this StateVariable without any modality."""
        return StateVariable(self.function, self.args, None, [])
    
    def get_args(self):
        """Return a list of arguments like they would be supplied to
        the Literal defining this StateVariable.

        If no modality is set, this will be the list of
        arguments. Otherwise, it will be the list of modal arguments
        with a FunctionTerm inserted at the appropriate Position."""
        if not self.modality:
            return self.args
        fterm = FunctionTerm(self.function, [ConstantTerm(a) for a in self.args])
        args = []
        it = iter(self.modal_args)
        for parg in self.modality.args:
            if isinstance(parg.type, FunctionType):
                args.append(fterm)
            else:
                args.append(it.next())
        return args

    def as_literal(self):
        """Return a representation of this StateVariable as a Literal.
        
        If this StateVariable's function is not a Predicate and no
        modality is defined, an Exception will be raised."""
        assert isinstance(self.function, Predicate) or self.modality is not None
        
        if not self.modality:
            return Literal(self.function, [ConstantTerm(a) for a in self.args])
        fterm = FunctionTerm(self.function, [ConstantTerm(a) for a in self.args])
        args = []
        it = iter(self.modal_args)
        for parg in self.modality.args:
            if isinstance(parg.type, FunctionType):
                args.append(fterm)
            else:
                args.append(ConstantTerm(it.next()))
        return Literal(self.modality, args)
        
    def __str__(self):
        s = "%s(%s)" % (self.function.name, " ".join(a.name for a in self.args))
        if self.modality:
            s = "%s(%s, %s)" % (self.modality.name, s, " ".join(a.name for a in self.modal_args))
        return s

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.hash == other.hash
        #return self.__class__ == other.__class__ and self.function == other.function and all(map(lambda a,b: a==b, self.args, other.args)) and self.modality == other.modality and all(map(lambda a,b: a==b, self.modal_args, other.modal_args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.hash
    
    @staticmethod
    def from_term(term, state=None):
        """Create a new StateVariable from a FunctionTerm. If the Term
        contains nested functions, a state must be applied to look
        them up.

        Arguments:
        term -- FunctionTerm
        state -- state to look up nested function."""
        assert isinstance(term, FunctionTerm)
        args = instantiate_args(term.args, state)
        return StateVariable(term.function, args)

    @staticmethod
    def get_svars_in_term(term, state=None):
        """Create a new StateVariable from a FunctionTerm and returns
        a list of all StateVariable that occur in this term (as nested
        functions). If the Term contains nested functions, a state
        must be applied to look them up.

        Returns a tuple with the resulting StateVariable and a list of
        contained StateVariables (including the result itself).

        Arguments:
        term -- FunctionTerm
        state -- state to look up nested function."""
        
        assert isinstance(term, FunctionTerm)
        svar, vars = get_svars_from_term(term, state)
        vars.add(svar)
        return svar, vars
    
    @staticmethod
    def from_literal(literal, state=None):
        """Create a new StateVariable from a Literal. If the Literal
        contains nested functions, a state must be applied to look
        them up.

        Arguments:
        literal -- Literal
        state -- state to look up nested function."""
        
        function = None
        litargs = []
        modal_args = []
        if literal.predicate in assignment_ops + [equals]:
            function = literal.args[0].function
            litargs = literal.args[0].args
            modal_args = None
        else:
            for arg, parg in zip(literal.args, literal.predicate.args):
                if isinstance(parg.type, FunctionType):
                    if function is None:
                        function = arg.function
                        litargs = arg.args
                    else:
                        assert False, "A modal svar can only contain one function"
                else:
                    modal_args.append(arg)

            if not function:
                function = literal.predicate
                litargs = literal.args
                modal_args = None

        args = instantiate_args(litargs, state)
        if modal_args is not None:
            modal_args = instantiate_args(modal_args, state)
            return StateVariable(function, args, literal.predicate, modal_args)
            
        return StateVariable(function, args)
    

class Fact(tuple):
    """This class represents an assignment of a value to a
    StateVariable. The values of StateVariables referring to
    predicates are TRUE and FALSE."""
    
    def __new__(_class, svar, value):
        """Create a new Fact.

        Arguments:
        svar -- StateVariable
        value -- TypedObject"""
        #assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        return tuple.__new__(_class, (svar, value))
    
    svar = property(lambda self: self[0])
    value = property(lambda self: self[1])
    
    def as_literal(self, useEqual=False, _class=None):
        """Return a representation of this Fact as a Literal."""
        
        if isinstance(self.svar.function, Predicate) or self.svar.modality is not None:
            lit = self.svar.as_literal()
            if self.value == TRUE:
                return lit
            elif self.value == FALSE:
                return lit.negate()
            
            assert False

        if useEqual:
            op = equal_assign
        else:
            op = assign
            
        l = Literal(op, [Term(self.svar.function, [Term(a) for a in self.svar.args]), Term(self.value)])
        if _class:
            l.__class__ = _class
        return l

    def __str__(self):
        return "%s = %s" % (str(self.svar), str(self.value))

    @staticmethod
    def from_literal(literal, state=None):
        """Create a new Fact from a Literal. If the Literal
        contains nested functions, a state must be applied to look
        them up.

        Arguments:
        literal -- Literal
        state -- state to look up nested function."""
        value = None
        if literal.predicate in assignment_ops + [equals]:
            value = instantiate_args(literal.args[-1:], state)[0]
        else:
            if literal.negated:
                value = FALSE
            else:
                value = TRUE

        return Fact(StateVariable.from_literal(literal, state), value)
        
    @staticmethod
    def from_condition(condition, state=None):
        """Return a list of Facts from a Condition. If Literals in the
        Condition contain nested functions, a state must be applied to
        look them up.

        This method only works for simple Conjunctions of
        Literals. For more complex conditions, use the is_satisfied()
        method of the State class.

        Arguments:
        condition -- Condition
        state -- state to look up nested function."""
        def factVisitor(cond, facts):
            if isinstance(cond, conditions.LiteralCondition):
                return [Fact.from_literal(cond, state)]
            elif isinstance(cond, conditions.Conjunction):
                return sum(facts, [])
            elif isinstance(cond, conditions.Truth):
                return []
            else:
                raise Exception("can't get facts for %s" % cond.__class__)
        return condition.visit(factVisitor)

    @staticmethod
    def from_effect(effect, state=None):
        """Return a list of Facts from an Effect. If Literals in the
        effect contain nested functions, a state must be applied to
        look them up.

        This method only works for simple conjunctions of
        Effect. For more complex conditions, use the get_effect_facts()
        method of the State class.

        Arguments:
        effect -- Effect
        state -- state to look up nested function."""
        def factVisitor(eff, facts):
            if isinstance(eff, effects.SimpleEffect):
                return [Fact.from_literal(eff, state)]
            elif isinstance(eff, effects.ConjunctiveEffect):
                return sum(facts, [])
            else:
                raise Exception("can't get facts for %s" % eff.__class__)
        return effect.visit(factVisitor)
    
    @staticmethod
    def from_tuple(tup):
        """Create a Fact object from a tuple containing a
        StateVariable and a TypedObject."""
        return Fact(tup[0], tup[1])
    
class State(dict):
    """This class represents a complete planning state. It is a
    dictionary of StateVariable to TypedObjects.

    It has additional features to compute the truth value of
    conditions and the results of effect statements, as well as the
    evaluation of axioms."""
    
    def __init__(self, facts=[], prob=None):
        """Create a new State object.

        Arguments:
        facts -- List of Fact objects or (StateVariable, TypedObject) tuples the State should be initialized with.
        prob -- The PDDL problem description this State is based on."""
        
        #defaultdict.__init__(self, lambda: UNKNOWN)
        assert prob is None or isinstance(prob, problem.Problem)
        self.problem = prob
        for f in facts:
            self.set(f)

        self.read_svars = set()
        self.written_svars = set()
        
        self.random = random.Random()

    def copy(self):
        """Create a copy of this State."""
        s = State([], self.problem)
        for svar,val in self.iteritems():
            s[svar] = val
        return s

    def set_random_seed(self, seed):
        """Set the random seed used to compute the effects of
        "probabilistic" statements.

        Argument:
        seed -- random seed"""
        self.random.seed(seed)

    def iterfacts(self):
        """Returns an iterator of all Facts contained in this
        State."""
        return (Fact.from_tuple(tup) for tup in self.iteritems())

    def set(self, fact):
        """Sets a StateVariable to a value as specified in the
        supplied Fact object."""
        if fact.value == UNKNOWN:
            if fact.svar in self:
                del self[fact.svar]
        else:
            self[fact.svar] = fact.value
        
    def __getitem__(self, key):
        try:
            return dict.__getitem__(self, key)
        except:
            if isinstance(key.function, Predicate):
                return FALSE
            return UNKNOWN
            
        
    def __setitem__(self, svar, value):
        assert isinstance(svar, StateVariable)
        assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        dict.__setitem__(self, svar, value)

    def __contains__(self, key):
        if isinstance(key, Fact):
            return dict.__contains__(self, key.svar) and self[key.svar] == key.value
        return dict.__contains__(self, key)

    def __str__(self):
        elems = sorted("%s = %s" % (str(k), v.name) for k,v in self.iteritems())
        return "\n".join(elems)

    @staticmethod
    def from_problem(problem, seed=None):
        """Create a State from a PDDL problem description.

        Arguments:
        problem -- a Problem object
        seed -- Random seed used to generate the state when the problem has probabilistic initial facts."""
        
        s = State([], problem)
        if seed is not None:
            s.set_random_seed(seed)
            
        for i in problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                s.apply_effect(i)
            else:
                s.set(Fact.from_literal(i))
        return s

    def evaluate_term(self, term, trace_vars=False):
        """Evaluate a Term and return its value in the current
        state. All Parameters in the term (and nested terms) must be
        instantiated, otherwise an Exception is raised.

        Arguments:
        term -- Term object to evaluate
        trace_vars -- if True, all StateVariables required to resolve
        the term are written to the read_svars member variable.
        """
        if term.__class__ == ConstantTerm:
            return term.object
        if term.__class__ == VariableTerm:
            assert term.is_instantiated(), "%s is not instantiated" % str(term)
            return term.get_instance()
        if term.__class__ == FunctionVariableTerm:
            assert term.is_instantiated(), "%s is not instantiated" % str(term)
            term = term.get_instance()

        values = []
        for arg in term.args:
            values.append(self.evaluate_term(arg, trace_vars))
        func = term.function

        if func == plus:
            return TypedNumber(values[0].value + values[1].value)
        elif func == minus:
            return TypedNumber(values[0].value - values[1].value)
        elif func == mult:
            return TypedNumber(values[0].value * values[1].value)
        elif func == div:
            return TypedNumber(values[0].value / values[1].value)
        elif func == neg:
            return TypedNumber(-values[0].value)
        
        svar = StateVariable(term.function, values)
        if trace_vars:
            self.read_svars.add(svar)
        return self[svar]

    def svar_from_term(self, term, trace_vars=False):
        """Create a StateVariable from a (potentially nested)
        FunctionTerm. All Parameters in the term (and nested terms)
        must be instantiated, otherwise an Exception is raised.

        Arguments:
        term -- Term object to evaluate
        trace_vars -- if True, all StateVariables required to resolve
        the term are written to the read_svars member variable.
        """
        
        assert isinstance(term, FunctionTerm), "%s is not a function term." % str(term)
        if isinstance(term, VariableTerm):
            assert term.is_instantiated(), "%s is not instantiated." % str(term)

        assert term.function not in numeric_functions, "can't create state variable form built-in function  %s." % term.function.name
        
        values = []
        for arg in term.args:
            values.append(self.evaluate_term(arg, trace_vars))
            
        return StateVariable(term.function, values)

    def evaluate_literal(self, literal, trace_vars=False):
        """Return the truth value of a Literal in this state. All
        Parameters to the literal (and nested terms) must be
        instantiated, otherwise an Exception is raised.

        Arguments:
        literal -- Literal object to evaluate
        trace_vars -- if True, all StateVariables required to resolve
        the literal's arguments are written to the read_svars member
        variable.
        """
        
#        print literal.pddl_str()
        values = []
        svars = []
        for arg, parg in zip(literal.args, literal.predicate.args):
            if isinstance(parg.type, FunctionType):
                svars.append(self.svar_from_term(arg, trace_vars))
            else:
                values.append(self.evaluate_term(arg, trace_vars))
        
        pred = literal.predicate
        if pred in (equals, eq):
            return (values[0].name == values[1].name) ^ literal.negated
        elif pred == gt:
            return (values[0].name > values[1].name) ^ literal.negated
        elif pred == lt:
            return (values[0].name < values[1].name) ^ literal.negated
        elif pred == ge:
            return (values[0].name >= values[1].name) ^ literal.negated
        elif pred == le:
            return (values[0].name <= values[1].name) ^ literal.negated
        else:
            if not svars:
                #no modal predicate:
                svar = StateVariable(pred, values)
            elif len(svars) == 1:
                #modal predicate:
                svar = svars[0].as_modality(pred, values)
            else:
                assert False, "A modal svar can only contain one function"
                
            if trace_vars:
                self.read_svars.add(svar)
#            print svar, self[svar], (self[svar] == TRUE) ^ literal.negated
            return (self[svar] == TRUE) ^ literal.negated


    def get_literal_effect(self, literal, trace_vars=False):
        """Return a Fact that describes the effect of applying a
        Literal to this state. Supported are adding/deleting atoms,
        fluent assignments and numeric operations. All Parameters to
        the literal (and nested terms) must be instantiated, otherwise
        an Exception is raised.

        Arguments:
        literal -- Literal object to apply.
        trace_vars -- if True, all StateVariables required to resolve
        the literal's arguments are written to the read_svars member
        variable.
        """
        
        eff_svar = None
        eff_value = None
        values = []
        svars = []
        for arg, parg in zip(literal.args, literal.predicate.args):
            if isinstance(parg.type, FunctionType):
                svars.append(self.svar_from_term(arg, trace_vars))
            else:
                values.append(self.evaluate_term(arg, trace_vars))
        
        pred = literal.predicate
        if pred in assignment_ops+numeric_ops:
            svar = svars[0]
            #hack:
            if svar.function == total_cost and svar not in self:
                self[svar] = TypedNumber(0.0)
            previous = self[svar]
            val = values[0]
            eff_svar = svar
        
        if pred in (assign, change, num_assign, equal_assign, num_equal_assign):
            eff_value = val
        elif pred == increase:
            eff_value = TypedNumber(previous.value + val.value)
        elif pred == decrease:
            eff_value = TypedNumber(previous.value - val.value)
        elif pred == scale_up:
            eff_value = TypedNumber(previous.value * val.value)
        elif pred == scale_down:
            eff_value = TypedNumber(previous.value / val.value)
        else:
            if not svars:
                #no modal predicate:
                svar = StateVariable(pred, values)
            elif len(svars) == 1:
                #modal predicate:
                svar = svars[0].as_modality(pred, values)
            else:
                assert False, "A modal svar can only contain one function"
                
            eff_svar = svar
            if literal.negated:
                eff_value = FALSE
            else:
                eff_value = TRUE
        return Fact(eff_svar, eff_value)
        
    def is_executable(self, action):
        """Returns True if the supplied action is executable in this
        state. All Parameters to the action must be instantiated,
        otherwise an Exception is raised.

        Arguments:
        action -- Action object to evaluate
        """
        extstate = self.get_extended_state(self.get_relevant_vars(action.precondition))
        return extstate.is_satisfied(action.precondition)
            
    def is_satisfied(self, cond, relevantVars=None, universal=None):
        """Returns True if the supplied Condition is satisfied in this
        state. All Parameters to the condition must be instantiated,
        otherwise an Exception is raised.

        Arguments:
        cond -- Condition object to evaluate
        relevantVars -- if a list is supplied, all StateVariables that
        were used to detemine the truth of the condition are written
        to that list.
        universal -- if a list is supplied, the Parameters of a
        UniversalCondition that is required to be satisfied will be
        written to that list.
        """
        import logging
        def instantianteAndCheck(cond, params):
            cond.instantiate(dict(zip(cond.args, params)))
            result = checkConditionVisitor(cond.condition)
            cond.uninstantiate()
            return result

        def allFacts(iterable):
            newfacts = []
            newuniversal = []
            for result, facts, universal in iterable:
                if not result:
                    return False, [], []
                newfacts += facts
                newuniversal += universal
            return True, newfacts, newuniversal

        def anyFacts(iterable):
            for result, facts, universal in iterable:
                if result:
                    return True, facts, universal
            return False, [], []
            
        def checkConditionVisitor(cond):
            if isinstance(cond, conditions.SoftGoalCondition):
                result, svars, univ = checkConditionVisitor(cond.cond)
                if not result:
                    return True, [], []
                return result, svars, univ
            if isinstance(cond, conditions.LiteralCondition):
                if relevantVars is not None:
                    self.read_svars.clear()
                    result = self.evaluate_literal(cond, trace_vars=True)
                    return result, list(self.read_svars), []
                
                return self.evaluate_literal(cond), [], []

            elif isinstance(cond, conditions.Truth):
                return True, [], []
            elif isinstance(cond, conditions.Falsity):
                return False, [], []
            elif isinstance(cond, conditions.Conjunction):
                if not cond.parts:
                    return True, [], []

                result = allFacts(imap(checkConditionVisitor, cond.parts))
                return result
            elif isinstance(cond, conditions.Disjunction):
                return anyFacts(imap(checkConditionVisitor, cond.parts))
            elif isinstance(cond, conditions.QuantifiedCondition):
                combinations = product(*map(lambda a: self.problem.get_all_objects(a.type), cond.args))
                if isinstance(cond, conditions.UniversalCondition):
                    result, vars, universal = allFacts(instantianteAndCheck(cond, c) for c in combinations)
                    universal += cond.args
                    return result, vars, universal
                elif isinstance(cond, conditions.ExistentialCondition):
                    return anyFacts(instantianteAndCheck(cond, c) for c in combinations)
            elif isinstance(cond, durative.TimedCondition):
                #ignore times specifiers for now
                return checkConditionVisitor(cond.condition)
            elif cond is None:
                return True, [], []
            assert False

        result, svars, univ = checkConditionVisitor(cond)

        if relevantVars is not None:
            relevantVars += svars
        if universal is not None:
            universal += univ
        return result

    def get_relevant_vars(self, cond, restrict_to=None):
        """Returns a list of StateVariables that might be relevant for
        determining the truth value of a condition.

        Arguments:
        cond -- the Condition object to evaluate
        restrict_to -- optionally a list of predicates for which the
        relevance should be checked."""
        def restrictionVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                return cond.predicate in restrict_to
            elif isinstance(cond, conditions.JunctionCondition):
                return any(imap(restrictionVisitor, cond.parts))
            elif isinstance(cond, conditions.QuantifiedCondition):
                return restrictionVisitor(cond.condition)
            else:
                return False

        def dependenciesVisitor(cond):
            if isinstance(cond, conditions.LiteralCondition):
                self.evaluate_literal(cond, trace_vars=True)

            elif isinstance(cond, conditions.JunctionCondition):
                for p in cond.parts:
                    dependenciesVisitor(p)
            elif isinstance(cond, conditions.QuantifiedCondition):
                if restrict_to:
                    if not restrictionVisitor(cond):
                        return
                    
                combinations = product(*map(lambda a: self.problem.get_all_objects(a.type), cond.args))
                result = []
                for c in combinations:
                    cond.instantiate(dict(zip(cond.args, c)))
                    dependenciesVisitor(cond.condition)
                    cond.uninstantiate()
                    
        self.read_svars.clear()
        dependenciesVisitor(cond)
        return set(self.read_svars)

    def get_effect_facts(self, effect, trace_vars=False):
        """Return list of Facts that describe the effect of applying
        an Effect to this state. Supported are adding/deleting atoms,
        fluent assignments and numeric operations. All Parameters to
        the Effect (and nested elements) must be instantiated,
        otherwise an Exception is raised.

        Arguments:
        effect -- Effect object to apply.
        trace_vars -- if True, all StateVariables required to resolve
        the effects's arguments are written to the read_svars member
        variable.
        """
        facts = {}
        if isinstance(effect, effects.UniversalEffect):
            combinations = product(*map(lambda a: self.problem.get_all_objects(a.type), effect.args))
            for params in combinations:
                effect.instantiate(dict(zip(effect.args, params)))
                facts.update(self.get_effect_facts(effect.effect, trace_vars))
                effect.uninstantiate()
                
        elif isinstance(effect, effects.ConditionalEffect):
            extstate = self.get_extended_state(self.get_relevant_vars(effect.condition))
            if extstate.is_satisfied(effect.condition):
                facts.update(self.get_effect_facts(effect.effect, trace_vars))

        elif isinstance(effect, effects.ProbabilisticEffect):
            remaining_effects = []
            p_total = 0.0
            s = self.random.random()
            
            for p, eff in effect.effects:
                assert p_total <= 1.0
                if p is None:
                    remaining_effects.append(eff)
                else:
                    p = self.evaluate_term(p, trace_vars=trace_vars)
                    if p_total <= s <= p_total + p.value:
                        return self.get_effect_facts(eff, trace_vars)
                    p_total += p.value

            if remaining_effects:
                p = (1.0-p_total) / len(remaining_effects)
                for eff in remaining_effects:
                    if p_total <= s <= p_total + p:
                        return self.get_effect_facts(eff, trace_vars)
                    p_total += p
            
        elif isinstance(effect, effects.ConjunctiveEffect):
            for eff in effect.parts:
                facts.update(self.get_effect_facts(eff, trace_vars))
            
        else:
            svar, val = self.get_literal_effect(effect, trace_vars)
            facts[svar] = val
        return facts
        
    def apply_effect(self, effect, trace_vars=False):
        """Apply an Effect to this state. Supported are
        adding/deleting atoms, fluent assignments and numeric
        operations. All Parameters to the Effect (and nested elements)
        must be instantiated, otherwise an Exception is raised.

        Arguments:
        effect -- Effect object to apply.
        trace_vars -- if True, all StateVariables required to resolve
        the effects's arguments are written to the read_svars member
        variable.
        """
        facts = self.get_effect_facts(effect, trace_vars=trace_vars)

        for svar, val in facts.iteritems():
            if trace_vars:
                self.written_svars.add(svar)
            self[svar] = val
            
    def get_extended_state(self, svars=None, getReasons=False):
        """Return a copy of this state with evaluated derived predicates.

        If getReasons is True, this method will additionally return a
        dictionary that contains for each derived StateVariable a list
        of non-derived StateVariables that were required to derive it.
        If evaluation of UniversalConditions is required for a
        derivation, the Parameters of those conditions will be
        supplied in a second directory.
        
        Arguments:
        svars -- If not None, only the axioms required to determine
        that value of the StateVariables in this list will be
        evaluated.
        getReasons -- also return from which variable each derived predicate was derived.
        """
        t0 = time.time()
        pred_to_axioms = defaultdict(set)
        for a in self.problem.axioms:
            pred_to_axioms[a.predicate].add(a)
        derived = pred_to_axioms.keys()

        def getDependencies(svar, derived):
            open = set([svar])
            closed = set()
            while open:
                sv = open.pop()
                closed.add(sv)
                for ax in self.problem.axioms:
                    if ax.predicate == sv.get_predicate():
                        ax.instantiate(sv.get_args())
                        for dep in self.get_relevant_vars(ax.condition, derived):
                            if dep.get_predicate() in derived and dep not in closed:
                                open.add(dep)
                        ax.uninstantiate()
            return closed
                
        relevant = set()
        if svars is not None:
            for s in svars:
                if s.get_predicate() in derived:
                    relevant |= getDependencies(s, derived)
            if not relevant:
                if getReasons:
                    return self, {}, {}
                return self


        if getReasons:
            reasons = defaultdict(set)
            universalReasons = defaultdict(set)

        #print "finding releveant:", time.time()-t0

        ex_state = State(self.iterfacts(), self.problem)

        import logging
        
        t0 = time.time()
        for level, preds in sorted(self.problem.stratification.iteritems()):
            logging.getLogger().debug("level: %d, %s", level, map(str, preds))
            t1 = time.time()
            axioms = set()
            #for p, ax in pred_to_axioms.iteritems():
            #    logging.getLogger().debug("%s => %s", p.name, ", ".join(a.predicate.name for a in ax))
                
            for p in preds:
                #logging.getLogger().debug("added %s", ", ".join(a.predicate.name for a in pred_to_axioms[p]))
                axioms |= pred_to_axioms[p]

            recursive_atoms = set()
            nonrecursive_atoms = set()
            true_atoms = set()
            for a in axioms:
                #check if the combination is actually valid, as sometimes a parameter type can depend
                #on a previous parameter (e.g. (typeof ?f) types)
                def is_valid(c):
                    return self.problem.predicates.get(a.predicate.name, c)
                
                if relevant:
                    gen = []
                    for svar in relevant:
                        atom = svar.as_literal()
                        if atom.predicate in preds:
                            gen.append(atom)
                else:
                    combinations = product(*map(lambda arg: list(self.problem.get_all_objects(arg.type)), a.args))

                    gen = (Literal(a.predicate, c, self.problem) for c in combinations if is_valid(c))
                    
                for atom in gen:
                    if a.predicate in self.problem.nonrecursive:
                        nonrecursive_atoms.add(atom)
                    else:
                        recursive_atoms.add(atom)

                    svar = StateVariable.from_literal(atom)
                    if svar in self and self[svar] == TRUE:
                        true_atoms.add(atom)

            #print "collecting level %d (%d atoms):" % (level, len(recursive_atoms)+len(nonrecursive_atoms)), time.time()-t1
            
            changed = True

            logging.getLogger().debug("start")
            while changed:
                logging.getLogger().debug("axiom eval pass")
                t2 = time.time()
                changed = False
                open = (nonrecursive_atoms | recursive_atoms) - true_atoms
                for atom in open:
                    #t3 = time.time()
                    for ax in pred_to_axioms[atom.predicate]:
                        if ax.tryInstantiate(atom.args):
                            if getReasons:
                                vars = []
                                universal = []
                            else:
                                vars = None
                                universal = None
                                
                            if ex_state.is_satisfied(ax.condition, vars, universal):
                                #logging.getLogger().debug("set to true: %s", atom.pddl_str())

                                true_atoms.add(atom)
                                ex_state[StateVariable.from_literal(atom)] = TRUE
                                changed = True
                                if getReasons:
                                    this_svar = StateVariable.from_literal(atom)
                                    for svar in vars:
                                        reasons[this_svar].add(svar)
                                    for param in universal:
                                        universalReasons[this_svar].add(param)
                                break
                                
                            ax.uninstantiate()
                    #print "atom:", time.time()-t3

                nonrecursive_atoms = set()
                #print "iteration:", time.time()-t2
            #print "layer:", time.time()-t1
        
        #print "total:", time.time()-t0
        if getReasons:
            return ex_state, reasons, universalReasons
        
        return ex_state
