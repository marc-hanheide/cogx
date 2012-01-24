#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools, random
from collections import defaultdict

import mapltypes as types
import problem, effects, mapl
from state import StateVariable, Fact, State
from builtin import *
from predicates import *

class ValueDistribution(defaultdict):
    def __init__(self, d=None, prob=1.0):
        defaultdict.__init__(self, lambda: 0.0)
        if isinstance(d, dict):
            self.update(d)
        elif isinstance(d, types.TypedObject):
            self[d] = prob

    def copy(self):
        return ValueDistribution(d=self)
    
    def _get_det_value(self):
        for val, p in self.iteritems():
            if p > 0.999:
                return val
        return None
    value = property(_get_det_value)

    def normalize(self):
        total = sum(v for k,v in self.iteritems() if k != UNKNOWN)
        if total > 1.0:
            norm = 1.0/total
            self *= norm
        elif total < 1.0:
            self[UNKNOWN] = 1.0-total

    def sample(self, _random=None):
        self.normalize()
        if not _random:
            _random = random.Random()
        s = _random.random()
        p_total = 0.0
        for v,p in sorted(self.iteritems(), key=lambda (v,p): p):
            p_total += p
            if s <= p_total:
                return v
        return v

    def assert_instanceof(self, svar):
        for val, p in self.iteritems():
            if p <= 0.0:
                continue
            assert val.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(val))

    def __iadd__(self, other):
        for k,v in other.iteritems():
            self[k] = self[k] + v
        return self

    def __add__(self, other):
        new = ValueDistribution(self)
        new += other
        return new

    def __imul__(self, other):
        for k,v in self.iteritems():
            self[k] *= other
        return self
    
    def __mul__(self, other):
        new = ValueDistribution(self)
        new *= other
        return new

    def __str__(self):
        return "(%s)" % ", ".join("%s: %s" % (v, p) for v,p in self.iteritems())
        
class ProbFact(Fact):
    def __new__(_class, svar, value):
        if isinstance(value, dict):
            vdist = ValueDistribution(value)
            vdist.assert_instanceof(svar)
            return tuple.__new__(_class, (svar, vdist))
        assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
        return tuple.__new__(_class, (svar, ValueDistribution(value)))
   
    def __str__(self):
        values = ["%s (%.2f)" % (str(k), v) for k,v in self.value.iteritems()]
        return "%s = {%s}" % (str(self.svar), ", ".join(values))

    def to_init(self):
        eff = self.to_effect()
        if isinstance(eff, Literal):
            if eff.predicate == assign:
                eff.predicate = equal_assign
            if eff.predicate == num_assign:
                eff.predicate = num_equal_assign
        return eff
        
    def to_effect(self):
        effect_tups = []
        for val, prob in self.value.iteritems():
            if prob > 0:
                eff = Fact(self.svar, val).as_literal(_class=effects.SimpleEffect)
                if prob == 1.0:
                    return eff
                effect_tups.append((Term(prob), eff))
                
        return effects.ProbabilisticEffect(effect_tups)

    @staticmethod
    def from_literal(lit, state=None):
        det_fact = Fact.from_literal(lit, state)
        return ProbFact(det_fact.svar, det_fact.value)
                
    @staticmethod
    def from_effect(effect, state=None):
        if isinstance(effect, effects.SimpleEffect):
            det_fact = Fact.from_literal(effect, state)
            return ProbFact(det_fact.svar, det_fact.value)
        elif isinstance(effect, effects.ProbabilisticEffect):
            remaining_effects = []
            p_total = 0

            for p, eff in effect.effects:
                assert p_total <= 1.0
                if p is None:
                    remaining_effects.append(eff)
                else:
                    if not isinstance(p, ConstantTerm):
                        assert s is not None
                        p = s.evaluate_term(p)
                    
                    p_total += p.object.value
            
            return ProbFact()#TODO
        raise  Exception("can't get facts for quantified or conditional effects")
    
    @staticmethod
    def from_tuple(tup):
        return ProbFact(tup[0], tup[1])

        
class ProbabilisticState(State):
    # def __init__(self, facts=[], prob=None):
    #     assert prob is None or isinstance(prob, problem.Problem)
    #     self.problem = prob
    #     for f in facts:
    #         self.set(f)

    #     self.read_svars = set()
    #     self.written_svars = set()
    #     self.extstate = None
    #     self.derived = set()
        
    #     self.random = random.Random()

    def copy(self):
        s = ProbabilisticState([], self.problem)
        for svar, values in self.iteritems():
            s[svar] = values.copy()
        return s

    def set_random_seed(self, seed):
        self.random.seed(seed)

    def iterdists(self):
        return (ProbFact.from_tuple(tup) for tup in self.iteritems())

    def iterfacts(self, only_nonzero=True):
        def fact_iter(svar, dist):
            for k,v in dist.iteritems():
                if v > 0 or not only_nonzero:
                    yield Fact(svar, k), v
        return itertools.chain(*(fact_iter(svar, dist) for svar,dist in self.iteritems()))

    def deterministic(self, threshold = None):
        for svar, val in self.iteritems():
            if val.value:
                yield Fact(svar, val.value)
            elif threshold is not None:
                assert threshold > 0.5
                for value, p in val.iteritems():
                    if p > threshold and value != UNKNOWN:
                        yield Fact(svar, value)

    def is_det(self, svar):
        if svar not in self:
            return True
        val = self[svar]
        if isinstance(val, types.TypedObject):
            return True
        return val.value is not None

    def prob(self, svar, value):
        if svar not in self:
            return 0.0
        val = self[svar]
        if isinstance(val, types.TypedObject):
            return 1.0
        return val[value]
    
    def set(self, fact):
        if isinstance(fact, ProbFact):
            self[fact.svar] = fact.value
        else:
            self[fact.svar] = ValueDistribution(fact.value)

    def update(self, fact):
        if not isinstance(fact, ProbFact):
            self.set(fact)
            return
        current = self.dist(fact.svar)
        updist = ValueDistribution(fact.value)
        current *= updist[UNKNOWN]
        del updist[UNKNOWN]
        current += updist
        self[fact.svar] = current

    def dist(self, key):
        try:
            return dict.__getitem__(self, key)
        except:
            if isinstance(key.function, Predicate):
                return ValueDistribution(FALSE)
            return ValueDistribution(UNKNOWN)
        
    def __getitem__(self, key):
        try:
            dist = State.__getitem__(self, key)
            dval = dist.value
            if dval is not None:
                return dval
            return dist
        except:
            if isinstance(key.function, Predicate):
                return FALSE
            return UNKNOWN
        
    def __setitem__(self, svar, value):
        assert isinstance(svar, StateVariable)
        if isinstance(value, dict):
            vdist = ValueDistribution(value)
            vdist.assert_instanceof(svar)
            dict.__setitem__(self, svar, vdist)
        else:
            assert value.is_instance_of(svar.get_type()), "type of %s (%s) is incompatible with %s" % (str(svar), str(svar.get_type()), str(value))
            dict.__setitem__(self, svar, ValueDistribution(value))

    def __contains__(self, key):
        #FIXME: This fails for default values (e.g. testing for P(x)=FALSE)
        if isinstance(key, Fact):
            return dict.__contains__(self, key.svar) and self.dist(key.svar)[key.value] > 0.0
        return dict.__contains__(self, key)

    @staticmethod
    def from_problem(problem, seed=None):
        s = ProbabilisticState([], problem)
        if seed is not None:
            s.set_random_seed(seed)
            
        for i in problem.init:
            if isinstance(i, effects.ProbabilisticEffect):
                s.apply_effect(i)
            else:
                s.set(Fact.from_literal(i))

        return s

    @staticmethod
    def sample_from_problem(problem, seed=None):
        st = State([], problem)
        if seed is not None:
            st.set_random_seed(seed)
            
        def descend(eff):
            if isinstance(eff, Literal):
                f = Fact.from_literal(eff)
                return {f : 1.0}
            if isinstance(eff, effects.ProbabilisticEffect):
                p_total = 0.0

                s = st.random.random()
                for p, e in eff.get_effects(st):
                    p_total += p
                    # print p_total, e.pddl_str()
                    if s <= p_total:
                        result = descend(e)
                        for fact, fp in result.iteritems():
                            result[fact] = fp * p
                        return result
                            
                return {}
            if isinstance(eff, (effects.ConjunctiveEffect, list)):
                parts = eff if isinstance(eff, list) else eff.parts
                
                result = defaultdict(ValueDistribution)
                assigned_vars = set()
                # branch_results = defaultdict(list)
                for e in parts:
                    done = False
                    while not done:
                        done = True
                        res = descend(e)
                        if any(f.svar in assigned_vars for f in res.iterkeys()):
                            # print "rejected:", map(str, res.iterkeys())
                            done = False # try until we get a consistent state
                        else:
                            for fact, p in res.iteritems():
                                assigned_vars.add(fact.svar)
                                result[fact] = p

                
                        
                # for svar, values in branch_results.iteritems():
                #     if len(values) == 1:
                #         val, p = values[0]
                #         result[Fact(svar, val)] = p
                #     else:
                #         #sample again:
                #         p_total = 0.0
                #         s = st.random.random() * sum(p for v,p in values)
                #         for v, p in values:
                #             p_total += p
                #             if s <= p_total:
                #                 result[Fact(svar, v)] = p
                #                 break
                # print map(str, result.iterkeys())
                return result
            assert False, eff

        result = descend(problem.init)
        for fact in result.iterkeys():
            st.set(fact)
        return st

    def sample(self, seed=None):
        print seed
        if seed is not None:
            self.set_random_seed(seed)

        facts = []
        for svar, dist in sorted(self.iteritems(), key=lambda (svar,dist): str(svar)):
            val = dist.sample(self.random)
            facts.append(Fact(svar, val))
            
        return State(facts, self.problem)


    def get_effect_facts(self, effect, trace_vars=False):
        import logging
        facts = defaultdict(ValueDistribution)
        
        if isinstance(effect, effects.ProbabilisticEffect):
            remaining_effects = []
            all_effects = []
            p_total = 0.0
            
            for p, eff in effect.effects:
                # assert p_total <= 1.0
                if p is None:
                    remaining_effects.append(eff)
                else:
                    p = self.evaluate_term(p, trace_vars=trace_vars)
                    all_effects.append((eff, p.value))
                    p_total += p.value

            if remaining_effects and p_total < 1.0:
                p = (1.0-p_total) / len(remaining_effects)
                for eff in remaining_effects:
                    all_effects.append((eff, p))

            normalize = 1.0
            if p_total > 1.0:
                logging.getLogger().warn("Probability distribution sum up to %.8f > 1: %s", p_total, effect.pddl_str())
                normalize = 1/p_total
                    
            for eff, p in all_effects:
                for svar, val in self.get_effect_facts(eff, trace_vars).iteritems():
                    if isinstance(val, ValueDistribution):
                        facts[svar] += val*p*normalize
                    else:
                        facts[svar][val] += p*normalize
                
            for dist in facts.itervalues():
                dist.normalize()
            return facts
        
        return State.get_effect_facts(self, effect, trace_vars)
        
    def apply_effect(self, effect, read_vars=None):
        if read_vars is not None:
            facts = self.get_effect_facts(effect, trace_vars=True)
            for svar in self.read_svars:
                read_vars.add(svar)
        else:
            facts = self.get_effect_facts(effect)

        for svar, val in facts.iteritems():
            if isinstance(val, ValueDistribution):
                self.update(ProbFact(svar, val))
            else:
                self[svar] = val

    def __str__(self):
        elems = sorted("%s = %s" % (str(k), str(v)) for k,v in self.iteritems())
        return "\n".join(elems)

    def determinized_state(self, rejection_ratio, upper_threshold):
        s = State(prob=self.problem)
        domains = defaultdict(list)
        exclude_domains = defaultdict(list)

        def get_p_svar(svar, value):
            func = self.problem.functions.get("p-%s" % svar.function.name, svar.args + (value,))
            if not func:
                return None
            return StateVariable(func, svar.args+(value,))

        for svar, dist in self.iterdists():
            total_p = 0
            dist.normalize()
            highest = max(dist.itervalues())
            lower_bound = highest * rejection_ratio
            if dist.value != UNKNOWN and svar.modality is None and not isinstance(svar.function, Predicate):
                dvar = svar.as_modality(mapl.defined)
                s[dvar] = TRUE
            for v,p in sorted(dist.items(), key=lambda (v,p): -p):
                # print svar, v, p
                if total_p >= upper_threshold or p < lower_bound:
                    exclude_domains[svar].append(v)
                    continue
                total_p += p
                if v == UNKNOWN:
                    exclude_domains.setdefault(svar,[])
                elif p >= upper_threshold:
                    s[svar] = v
                else:
                    domains[svar].append((p, v))
                
#         for fact, prob in self.iterfacts(only_nonzero=False):
#             #TODO: Handle cases with limited number of alternatives
#             if fact.value == UNKNOWN and prob > lower_threshold:
#                 exclude_domains.setdefault(fact.svar,[])
#             elif prob >= upper_threshold:
#                 s.set(fact)
#             elif prob >= lower_threshold:
# #                idvar = fact.svar.as_modality(mapl.not_indomain, [fact.value])
#                 domains[fact.svar].append((prob, fact.value))
#             else:
#                 exclude_domains[fact.svar].append(fact.value)
                
        for svar, values in domains.iteritems():
            if svar in s and s[svar] != UNKNOWN:
                #already known
                continue
            for p,v in values:
                idvar = svar.as_modality(mapl.i_indomain, [v])
                s[idvar] = TRUE
                pvar = get_p_svar(svar, v)
                if pvar:
                    s[pvar] = p

        for svar, excluded in exclude_domains.iteritems():
            if svar in s and s[svar] != UNKNOWN:
                #already known
                continue
            if svar in domains:
                #already handled in the previous loop
                continue
            excluded = set(excluded)
            pvars = []
            for v in self.problem.get_all_objects(svar.get_type()):
                if v in excluded:
                    continue
                idvar = svar.as_modality(mapl.i_indomain, [v])
                s[idvar] = TRUE
                pvar = get_p_svar(svar, v)
                if pvar:
                    pvars.append(pvar)
            if pvars:
                p = types.TypedNumber(1.0/len(pvars))
                for pvar in pvars:
                    s[pvar] = p

        return s
            
        
