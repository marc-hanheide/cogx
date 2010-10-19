import math
from collections import defaultdict
from itertools import chain

from standalone import config, pddl
from standalone.pddl import state

log = config.logger("PythonServer")

class ProblemConstraint(object):
    def matches(self, obj, st):
        raise NotImplementedError()

    def __hash__(self):
        return self.hash
    def __eq__(self, o):
        return self.__class__ == o.__class__ and self.hash == o.hash

class ObjectsConstraint(object):
    def __init__(self, objects):
        self.objects = frozenset(objects)
        self.types = set(o.type for o in self.objects)
        self.hash = hash(self.objects)

    def matches(self, obj, st):
        if obj.type in self.types:
            return obj in self.objects
        return True

    def __str__(self):
        return ", ".join(o.name for o in self.objects)

class FunctionConstraint(object):
    def __init__(self, function, args, values):
        self.function = function
        self.args = args
        assert len([a for a in args if a.name.startswith('any_')]) == 1
        self.values = frozenset(values)
        self.types = set(a.type for a in self.args if a.name.startswith('any_'))
        self.hash = hash((self.function, self.values) + tuple(self.args))

    def matches(self, obj, st):
        if not any(obj.is_instance_of(t) for t in self.types):
            return True
        
        newargs = []
        for a in self.args:
            if a.name.startswith("any_"):
                assert obj.is_instance_of(a.type)
                newargs.append(obj)
            else:
                newargs.append(a)
        newvar = state.StateVariable(self.function, newargs)
        return st[newvar] in self.values

    def __str__(self):
        return "(%s %s) = [%s]" % (self.function.name, " ".join(a.name for a in self.args), ", ".join(v.name for v in self.values))
    
    def generalize(self, objects):
        newvals = set(o for o in objects if o.is_instance_of(self.function.type))
        return FunctionConstraint(self.function, self.args, newvals)

def all_objects(arg, objects):
    if isinstance(arg, pddl.Parameter):
        return [o for o in objects if o.is_instance_of(arg.type)]
    if arg in objects:
        return arg
    return None
    
class PartialProblem(object):
    def __init__(self, pstate, objects, min_constraints, max_constraints, rules, domain):
        self.state = pstate
        self.detstate = pstate.determinized_state(0.05, 0.95)
        self.objects = objects
        self.min_constraints = min_constraints
        self.max_constraints = max_constraints
        self.constraints = min_constraints
        self.rules = rules
        self.domain = domain
        self.objects, self.facts = self.limit(min_constraints)
        self.min_objects, _ = self.limit(max_constraints)
        
        self.initialize(rules, domain)
        self.reduction_order = None

    def __str__(self):
        maxstr = ", ".join(o.name for o in self.objects)
        minstr = ", ".join(o.name for o in self.min_objects)

        return "min: %d (%s), max: %d (%s)" % (self.min_size, minstr, self.max_size, maxstr)
        
    def initialize(self, rules, domain):
        self.objects_by_type = {}
        self.alt_objects_by_type = {}
        for t in domain.types.itervalues():
            self.objects_by_type[t] = set(o for o in self.min_objects if o.is_instance_of(t) and o != pddl.UNKNOWN)
            self.alt_objects_by_type[t] = set(o for o in self.objects if o.is_instance_of(t) and o != pddl.UNKNOWN)

        # print map(str, self.objects)
        # for t, objs in self.objects_by_type.iteritems():
        #     print "%s: %d (%s)" % (t.name, len(objs), ", ".join(o.name for o in objs))
            
        def get_rule_size(r, alt=False):
            if alt:
                d = self.alt_objects_by_type
            else:
                d = self.objects_by_type
            values = set(v for p,v in r.values)
            num_instances = 1
            for a in r.args:
                if isinstance(a, pddl.Parameter):
                    num_instances *= len(d[a.type])
            num_values = 0
            for v in values:
                if isinstance(v, pddl.VariableTerm):
                    num_values += len(d[v.object.type]) + 1 # add 1 for remaining possibilities
                else:
                    num_values += 1
            return num_instances, num_values

        total_count = 1
        total_count_max = 1
        for r in rules:
            inst, values = get_rule_size(r)
            total_count *= values**inst
            inst, values = get_rule_size(r, alt=True)
            total_count_max *= values**inst
            #print r, inst, values

        self.min_size = total_count
        self.max_size = total_count_max

    def get_objects(self, type):
        return self.alt_objects_by_type.get(type, [])

    def limit(self, constraints):
        result = set()
        for o in self.objects:
            if all(c.matches(o, self.detstate) for c in constraints):
                result.add(o)

        def check_value(val):
            if val.type == pddl.t_number:
                return True
            return val in result
                
        facts = []
        for f in self.state.iterdists():
            if not all(a in result for a in f.svar.args + f.svar.modal_args):
                continue
            
            vdist = pddl.prob_state.ValueDistribution(dict((d,p) for d,p in f.value.iteritems() if check_value(d)))
            vdist.normalize()
            # if len(vdist) != len(f.value):
            #     log.debug("Possible values were left out for %s!", str(f.svar))
            facts.append(pddl.prob_state.ProbFact(f.svar, vdist))
        return result, facts

    def compute_removal_criterions(self, to_remove, remaining):
        types = set(o.type for o in to_remove)
        fixed_types = set(o.type for o in remaining)

        # the value to remove occurs as a value of an already fixed feature
        value_order_rules = defaultdict(set)
        # the value to remove occurs as a parameter together with an already fixed feature
        corr_order_rules = defaultdict(set)
        for t in types:
            for r in self.rules:
                for _,v in r.values:
                    if t.equal_or_subtype_of(v.get_type()):
                        value_order_rules[t].add(r)
                        break
                for a in r.args:
                    #TODO: this doesn't take the rule condition into account at all.
                    if t.equal_or_subtype_of(a.type):
                        if any(ft.equal_or_subtype_of(a2.type) for ft in fixed_types for a2 in r.args if a2 != a):
                            corr_order_rules[t].add(r)
                            break
        return value_order_rules, corr_order_rules

    def compute_removal_order(self, hstate):
        obj_constraints = []
        new_constraints = []
        for c in self.max_constraints:
            if isinstance(c, ObjectsConstraint):
                obj_constraints.append(c)
            elif c not in self.min_constraints:
                new_constraints.append(c)

        remaining = set(o for o in self.objects if all(c.matches(o, self.detstate) for c in obj_constraints))
        to_remove = set(o for o in self.objects if not all(c.matches(o, self.detstate) for c in obj_constraints))
        log.debug("candidates for removal: %s", ", ".join(a.name for a in to_remove))
        assert to_remove
        vo_rules, co_rules = self.compute_removal_criterions(to_remove, remaining)

        sorted_by_type = dict((t, [o for o in to_remove if o.type == t]) for t in chain(vo_rules.iterkeys(), co_rules.iterkeys()))
        for t, rules in vo_rules.iteritems():
            log.debug("ordering %s by probability", str(t))
            functions = set(r.function for r in rules)
            obj_by_prob = []
            for o in to_remove:
                if t != o.type:
                    continue
                p_total = 0
                p_count = 0
                for f in functions:
                    combinations = state.product(*map(lambda a: all_objects(a, remaining), f.args))
                    for c in combinations:
                        svar = state.StateVariable(f, c)
                        p = hstate.get_prob(svar, o)
                        p_total += p
                        p_count +=1
                p_avg = p_total/p_count
                obj_by_prob.append((o,p_avg))
            sorted_by_type[t] = sorted(obj_by_prob, key=lambda (o,p): p)
            
        for t, rules in co_rules.iteritems():
            log.debug("ordering %s by disambiguation power", str(t))
            obj_entropies = defaultdict(lambda: 0.0)
            obj_counts = defaultdict(lambda: 0)
            
            for r in rules:
                for lit in r.conditions:
                    if lit.predicate == pddl.equals:
                        cterm = lit.args[0]
                    else:
                        cterm = pddl.Term(lit.predicate, lit.args)

                    cond_combinations = state.product(*map(lambda a: all_objects(a.object, remaining), cterm.args))
                    for c in cond_combinations:
                        var_combinations = state.product(*map(lambda a: all_objects(a, remaining|to_remove), r.args))
                        condvar = state.StateVariable(cterm.function, c)
                        for c2 in var_combinations:
                            depvar = state.StateVariable(r.function, c2)
                            jointp = hstate.get_joint_prob([condvar, depvar])
                            dep_p = defaultdict(lambda: 0.0)
                            for vals, p in jointp:
                                dep_p[vals[1]] += p
                            H = 0
                            for vals, p in jointp:
                                H += p * math.log(dep_p[vals[1]]/p,2)
                            log.debug("H(%s|%s) = %.6f", str(condvar), str(depvar), H)
                            #TODO: only insert "correct" variable, too late now.
                            for o in c2:
                                if o in to_remove:
                                    obj_entropies[o] += H
                                    obj_counts[o] += 1
                                    
            obj_by_entropy = []
            for o, H in obj_entropies.iteritems():
                #TODO: the averaged entropies are probably not the best way to select the objects
                obj_by_entropy.append((o, H/obj_counts[o]))
                        
            for o in to_remove:
                if o not in obj_entropies:
                    obj_by_entropy.append((o, 1.0))
            sorted_by_type[t] = sorted(obj_by_entropy, key=lambda (o,h): -h)

        result = []
        for t in vo_rules.iterkeys():
            result += [o for o,p in sorted_by_type[t]]
        for t in co_rules.iterkeys():
            result += [o for o,p in sorted_by_type[t]]
        return result

    def reduce(self, hstate):
        if self.reduction_order is None:
            self.reduction_order = self.compute_removal_order(hstate)

        if not self.reduction_order:
            return False
            
        #log.debug("objects: %s", ", ".join(o.name for o in self.objects))
        rem = self.reduction_order.pop(0)
        log.debug("removing: %s", rem.name)

        cnew = []
        for c in self.max_constraints:
            if isinstance(c, ObjectsConstraint):
                cnew.append(ObjectsConstraint(c.objects | set(self.reduction_order)))
            else:
                cnew.append(c.generalize(self.min_objects | set(self.reduction_order)))
        self.constraints = cnew

        # for c in cnew:
        #     print c

        self.old_objects = set(self.objects)
        self.objects, self.facts = self.limit(cnew)
        #self.initialize(self.rules, self.domain)
        self.max_size = hstate.size()
        #log.debug("remaining objects: %s", ", ".join(o.name for o in self.objects))
        log.debug("size is now: %d", self.max_size)

        def delete_object(obj, st):
            for svar in st.keys():
                if obj in svar.args + svar.modal_args:
                    del st[svar]
            for d in st.substates.itervalues():
                for p, sub in d.itervalues():
                    delete_object(obj, sub)

        for removed in self.old_objects - self.objects:
            delete_object(removed, hstate)
        #import debug
        #debug.set_trace()
        hstate.cleanup()
        return True

        #TODO: do the actual state reduction
        
        #self.max_size = 1
        
