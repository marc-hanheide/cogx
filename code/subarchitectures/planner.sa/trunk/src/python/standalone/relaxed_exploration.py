import sys, time, random
from itertools import chain, product, izip

from collections import defaultdict
from standalone import pddl
from standalone.pddl import state, conditions, dtpddl, mapl, translators, visitors, effects


def instantiation_function(action, problem, check_callback, force_callback, clear_callback):
    def args_visitor(term, results):
        if isinstance(term, pddl.FunctionTerm):
            return sum(results, [])
        return [term]

    cond_by_arg = defaultdict(set)
    free_args = {}

    def subcond_visitor(cond, result):
        for arg in cond.free():
            cond_by_arg[arg].add(cond)
        if isinstance(cond, conditions.LiteralCondition):
            free_args[cond] = cond.free()

    condition = action.precondition

    visitors.visit(condition, subcond_visitor)

    prev_mapping = {}
    checked = set()

    def inst_func(mapping, args):
        next_candidates = []
        if checked:
            for k,v in prev_mapping.iteritems():
                if mapping.get(k, None) != v:
                    checked.difference_update(cond_by_arg[k])
                    for cond in cond_by_arg[k]:
                        clear_callback(cond)
        prev_mapping.update(mapping)

        def instantianteAndCheck(cond, combinations, func):
            for c in combinations:
                cond.instantiate(dict(zip(cond.args, c)), problem)
                result = func()
                cond.uninstantiate()
                yield result
        
        #print [a.name for a in mapping.iterkeys()]
        forced = []
        def check(cond):
            if not cond or cond in checked:
                return True
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == pddl.equals and isinstance(cond.args[0], pddl.FunctionTerm) and isinstance(cond.args[1], pddl.VariableTerm):
                    v = cond.args[-1]
                    if all(a.is_instantiated() for a in cond.args[0].args if isinstance(a, pddl.VariableTerm)) and  isinstance(v, pddl.VariableTerm) and not v.is_instantiated():
                        svar = state.StateVariable.from_literal(cond)
                        val = force_callback(svar, cond)
                        if val:
                            forced.append((v.object, val, cond))
                        elif val == pddl.UNKNOWN:
                            return False

                if all(a.is_instantiated() for a in free_args[cond]):
                    fact = state.Fact.from_literal(cond)
                    if check_callback(fact, cond):
                        checked.add(cond)
                        return True
                    return False
                    # exst = st.get_extended_state([fact.svar])
                    # #TODO: handle all possible conditions
                    # if exst[fact.svar] != fact.value:
                    #     return False
                    # checked.add(cond)
                    # return True
                else:
                    next_candidates.append([a for a in free_args[cond] if not a.is_instantiated()])
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
                combinations = product(*map(lambda a: list(problem.get_all_objects(a.type)), cond.args))
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


class EffectGenerator(object):
    def __init__(self, actions):
        self.actions = list(actions)
        self.funcdict = defaultdict(list)
        for a in actions:
            self.prepare_dict(a)

    def prepare_dict(self, action):
        for lit in visitors.visit(action.effect, visitors.collect_literals, []):
            function_arg = None
            if any(isinstance(a, pddl.predicates.FunctionVariableTerm) for a in lit.args):
                modality = lit.predicate
                function = None
                args = None
                modal_args = []
                for a in lit.args:
                    if isinstance(a, pddl.predicates.FunctionVariableTerm):
                        function_arg = a.object
                    else:
                        modal_args.append(a.object)
                value = pddl.TRUE if not lit.negated else pddl.FALSE
            else:
                function, args, modality, modal_args, value = state.StateVariable.svar_args_from_literal(lit)
                args = [a.object for a in args]
                modal_args = [a.object for a in modal_args] if modal_args else []
                value = value.object

            if value.type == pddl.t_number:
                continue

            self.funcdict[(modality, function)].append((args, modal_args, value, function_arg, action))
                
            # if lit.predicate in pddl.assignment_ops:
            #     function = lit.args[0].function
            #     svar_args = [a.object for a in lit.args[0].args]
            #     val_arg = lit.args[1].object
            # elif not any (isinstance(a, pddl.FunctionTerm) for a in lit.args):
            #     function = lit.predicate
            #     svar_args = [a.object for a in lit.args]
            #     val_arg = pddl.FALSE if lit.negated else pddl.TRUE
            # else:
            #     print "ignore", action.name, lit.pddl_str()
            #     continue
            # # print function, map(str, svar_args), val_arg, action.name
            # self.funcdict[(modality, function)].append((svar_args, modal_args, val_arg, action))

    def action_from_fact(self, fact):
        #(modality, None) contains modal actions
        entries = chain(self.funcdict[(fact.svar.modality, fact.svar.function)], self.funcdict[(fact.svar.modality, None)])
        for svar_args, modal_args, val_arg, func_arg, action in entries:
            mapping = {}
            #modal action:
            if func_arg is not None:
                assert fact.svar.modality is not None, fact
                mapping[func_arg] = fact.svar.nonmodal().as_term()
                args_map = chain(zip(modal_args, fact.svar.modal_args), [(val_arg, fact.value)])
            else:
                args_map = chain(zip(svar_args, fact.svar.args), zip(modal_args, fact.svar.modal_args), [(val_arg, fact.value)])
                
            for arg, val in args_map:
                if isinstance(arg, pddl.Parameter) and val.is_instance_of(arg.type):
                    mapping[arg] = val
                elif arg != val:
                    mapping = None
                    break
            if mapping:
                # print "gen:", fact, action.name, map(str, mapping.values())
                yield action, mapping

class UnaryOp(object):
    def __init__(self, action, args, effect, preconds, all_preconds):
        self.action = action
        self.args = args
        self.effect = effect
        self.preconds = preconds
        self.all_preconds = all_preconds
        self.unsat_preconds = len(preconds)
        self.expanded = False
        self.initialised = False
        self.key = None

    def __str__(self):
        return "(%s %s) => %s" % (self.action.name, " ".join(str(a) for a in self.args), str(self.effect))

    @staticmethod
    def unify_with_parent(c_ops, action, args, preconds, st):
        mapping = dict(zip(action.args, args))
        for op in c_ops:
            if all(mapping[action[a.name]] == val for a, val in zip(op.action.args, op.args)):
                # print "merge", op, action.name, map(str,args)
                all_preconds = preconds | op.all_preconds
                unsat_preconds = set(f for f in all_preconds if f not in st)
                yield UnaryOp(action, args, op.effect, unsat_preconds, all_preconds)
        
    @staticmethod
    def build_ops(action, args, preconds, st):
        unsat_preconds = set(f for f in preconds if f not in st)
        found = False
        for lit in visitors.visit(action.effect, visitors.collect_literals, []):
            if lit.predicate in pddl.builtin.numeric_ops:
                continue
            eff =  state.Fact.from_literal(lit)
            found = True
            yield UnaryOp(action, args, eff, unsat_preconds, preconds)
        if not found:
            yield UnaryOp(action, args, None, unsat_preconds, preconds)
            

def action_from_axiom(axiom, domain):
    action = pddl.Action("axiom_%s" % axiom.predicate.name, [], None, None, domain)
    action.args = action.copy_args(axiom.args)
    action.effect = effects.SimpleEffect(axiom.predicate, action.args, action)

    def extract_exists(cond, results):
        if results:
            new_args, results = zip(*results)
            # print new_args, results
        else:
            new_args = []

        # print cond.pddl_str(), results
        new_args = sum(new_args, [])
        if isinstance(cond, conditions.ExistentialCondition):
            return new_args + cond.args, results[0]
        elif isinstance(cond, conditions.UniversalCondition):
            return [], cond.copy()
        
        return new_args, cond.copy(new_parts = filter(None, results))

    new_args, cond = axiom.condition.visit(extract_exists)
    action.args += action.copy_args(new_args)
    action.precondition = cond
    action.precondition.set_scope(action)
    return action

def extract_exists(action, domain):
    if not action.precondition:
        return action
    
    def extract_exists(cond, results):
        if results:
            new_args, results = zip(*results)
            # print new_args, results
        else:
            new_args = []

        # print cond.pddl_str(), results
        new_args = sum(new_args, [])
        if isinstance(cond, conditions.ExistentialCondition):
            return new_args + cond.args, results[0]
        elif isinstance(cond, conditions.UniversalCondition):
            return [], cond.copy()
        
        return new_args, cond.copy(new_parts = filter(None, results))
    
    new_args, cond = action.precondition.visit(extract_exists)
    if new_args:
        a2 = action.copy()
        a2.args += a2.copy_args(new_args)
        # print map(str, a2.args)
        a2.precondition = cond
        a2.precondition.set_scope(a2)
        return a2
    return action

def split_disjunction(action, domain, nonstatic):

    def get_disjunctions(cond, result):
        # print "---",cond.pddl_str()
        if not result:
            return [cond.copy()]
        
        if isinstance(cond, conditions.Disjunction):
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

    disjunctive_parts = pddl.visitors.visit(action.precondition, get_disjunctions, [])
    if len(disjunctive_parts) > 1:
        eff_args = action.effect.free() if action.effect else set()
        result = []
        for i, dis in enumerate(disjunctive_parts):
            # print dis.pddl_str()
            new_args = dis.free() | eff_args
            # print map(str, new_args)
            da = pddl.Action("%s_%d" % (action.name, i) , [], None, None, domain)
            da.args = da.copy_args([a for a in action.args if a in new_args])
            da.precondition = dis
            da.precondition.set_scope(da)
            da.effect = action.effect.copy(new_scope=da)
            result.append(da)
            # print da.name, dis.pddl_str()
        return result
    return [action]
        
            
def instantiate(actions, start, stat, domain, start_actions=[]):
    def get_nonstatic(a):
        return visitors.visit(a.effect, visitors.collect_non_builtins, [])

    new_id = [0]
    cond_actions = {}
    def create_conditional_effects(action):
        @pddl.visitors.collect
        def get_ceffs(elem, results):
            if isinstance(elem, effects.ConditionalEffect):
                return elem

        ceffs = pddl.visitors.visit(action.effect, get_ceffs, [])
        if isinstance(action, mapl.MAPLAction) and action.sensors:
            ceffs += action.conditional_knowledge_effect().visit(get_ceffs)

        for ceff in ceffs:
            ca = pddl.Action("%s_ceff_%d" % (action.name, new_id[0]) , [], None, None, domain)
            new_id[0] += 1
            ca.args = ca.copy_args(set(ceff.visit(pddl.visitors.collect_free_vars)))
            ca.precondition = ceff.condition.copy(new_scope = ca)
            ca.effect = ceff.effect.copy(new_scope = ca)

            cond_actions[ca] = action
            yield ca

    actions = actions + [action_from_axiom(a, domain) for a in domain.axioms]
    for ga, _ in start_actions:
        if ga not in actions:
            actions.append(ga)
    
    nonstatic = set(sum((get_nonstatic(a) for a in actions), [])) | set([mapl.hyp, mapl.knowledge, mapl.direct_knowledge, mapl.indomain, mapl.i_indomain])

    new_start_actions = []
    new_actions = []
    for a in actions:
        ex_a = extract_exists(a, domain)
        res = split_disjunction(ex_a, domain, nonstatic)
        new_actions += res
        for sa, mapping in start_actions:
            if a == sa:
                for new in res:
                    new_start_actions.append((new, mapping))
    start_actions = new_start_actions
    actions = new_actions
    
    actions += sum((list(create_conditional_effects(a)) for a in actions), [])

    # for a, map in start_actions:
    #     print a.name, map
    #     print a.precondition.pddl_str()
    
    effect_gen = EffectGenerator(actions)
    def get_objects(arg):
        return list(stat.problem.get_all_objects(arg.type))

    fact_by_cond = defaultdict(set)
    def check_func(fact, cond):
        # if fact.svar.modality == pddl.mapl.hyp and prob_state is not None:
        #     svar = fact.svar.nonmodal()
        #     val = fact.svar.modal_args[0]
        #     if prob_state.is_det(svar):
        #         return prob_state[svar] == val
        #     return prob_state[svar][val] > 0
        if fact.svar.modality == pddl.mapl.commit:
            return True
        if (fact.svar.function in nonstatic and not fact.svar.modality) or fact.svar.modality in nonstatic:
            fact_by_cond[cond].add(fact)
            return True
        else:
            # exst = stat.get_extended_state([fact.svar])
            #TODO: handle all possible conditions
            if stat[fact.svar] != fact.value:
                return False
            return True

    def all_facts():
        for fs in fact_by_cond.itervalues():
            for fact in fs:
                yield fact

    def clear_func(cond):
        fact_by_cond[cond] = set()
            
    def force_func(svar, cond):
        if svar.function in nonstatic:
            return None
        else:
            return stat[svar]

    waiting_cond_ops = defaultdict(list)
    unary_successors = defaultdict(list)
    applicable_ops = []
    def add_unary(action, args, preconds):
        uops_gen = UnaryOp.build_ops(action, args, preconds, stat)
        if action in waiting_cond_ops:
            uops_gen = chain(uops_gen, UnaryOp.unify_with_parent(waiting_cond_ops[action], action, args, preconds, stat))
            
        for uop in uops_gen:
            # print uop, map(str, uop.preconds)
            # print map(str, uop.preconds), "=>", uop, uop.unsat_preconds
            if action not in cond_actions:
                for pre in uop.all_preconds:
                    unary_successors[pre].append(uop)
                if not uop.preconds:
                    unary_successors[None].append(uop)
                if uop.unsat_preconds == 0:
                    applicable_ops.append(uop)
            else:
                # print "waiting:", uop
                waiting_cond_ops[cond_actions[action]].append(uop)

                

    action_predecessors = defaultdict(set)
    goal_actions = set()
    
    open = ["start"] + list(start)
    closed = set()
    closed_actions = set()
    all_actions = set()
    active_cond_actions = []
    while open:
        if active_cond_actions:
            next_actions = active_cond_actions
            active_cond_actions = []
        else:
            fact = open.pop(0)
            if fact == "start":
                next_actions = start_actions
            else:
                next_actions = effect_gen.action_from_fact(fact)
        # print "*",fact
            
        for action, mapping in next_actions:
            # print action.name, map(str, (mapping.get(a,a) for a in action.args))
            action_key  = (action, tuple(mapping.get(a,a) for a in action.args))
            if  action_key in closed_actions:
                continue
            closed_actions.add(action_key)
            
            fact_by_cond.clear()
            if any(isinstance(a.type, pddl.types.ProxyType) for a in action.args):
                action.instantiate(mapping, stat.problem)
                arg_lists = [get_objects(a) for a in action.args]
                action.uninstantiate()
            else:
                arg_lists = [get_objects(a) for a in action.args]
                
            func = instantiation_function(action, stat.problem, check_func, force_func, clear_func)
            for full_mapping in action.smart_instantiate(func, action.args, arg_lists, stat.problem, mapping):
                action_key  = (action, tuple(full_mapping.get(a,a) for a in action.args))
                if fact == "start":
                    goal_actions.add(action_key)
                if action_key in all_actions:
                    continue

                all_actions.add(action_key)
                relevant_facts = set(all_facts())
                add_unary(action, action_key[1], relevant_facts)

                if action in cond_actions:
                    dep_action = cond_actions[action]
                    dep_mapping = dict((dep_action[arg.name], val) for arg, val in full_mapping.iteritems())
                    # print "DEP:",dep_action.name, map(str, dep_action.args), ["%s => %s" % (str(k), str(v)) for k,v in dep_mapping.iteritems()]
                    active_cond_actions.append((dep_action, dep_mapping))
                
                # print "relevant: (%s %s)" % (action.name, " ".join(str(full_mapping.get(a,a)) for a in action.args))
                for pred_fact in relevant_facts:
                    if pred_fact not in closed:
                        # print "next:", pred_fact
                        closed.add(pred_fact)
                        open.append(pred_fact)

    for prop, ops in unary_successors.iteritems():
        new_ops = set()
        for o in ops:
            if o.effect in closed or (o.action, o.args) in goal_actions:
                new_ops.add(o)
        unary_successors[prop] = new_ops

    # for (action, args), pred in action_predecessors.iteritems():
    #     print "%s => (%s %s)" % (", ".join(str(f) for f in pred), action.name, " ".join(a.name for a in args))
    # print len(all_actions)
    return unary_successors, applicable_ops

def explore(actions, start, stat, domain, start_actions=[], prob_state=None):
    unary_successors, applicable_ops = instantiate(actions, start, stat, domain, start_actions)
    
    def is_goal_action(op):
        for action, mapping in start_actions:
            if op.action.name == action.name:
                # print op, [mapping.get(a,a2).name for a,a2 in zip(action.args, op.args)], all(mapping.get(a,a2) == a2 for a,a2 in zip(action.args, op.args))
                if all(mapping.get(a,a2) == a2 for a,a2 in zip(action.args, op.args)):
                    return True
        return False
    
    goal = set(start)
    facts = set(goal)
    actions = set()
    
    reached_by = {}
    forward_open = applicable_ops[:]
    # print map(str, applicable_ops)
    forward_closed = set(forward_open)
    while forward_open:
        next = forward_open.pop(0)
        if next.effect in reached_by:
            continue
        next.expanded = True

        if is_goal_action(next):
            # print "goal!"
            goal |= next.preconds
            facts |= next.all_preconds
            actions.add((next.action, next.args))
        
        # print next#, "=>", map(str, unary_successors[next.effect])
        if next.effect:
            reached_by[next.effect] = next

        for succ in unary_successors[next.effect]:
            if succ.expanded:
                continue
            # print "  ", succ, next.effect, succ.unsat_preconds 

            succ.unsat_preconds -= 1
            if succ.unsat_preconds == 0 and succ.effect not in forward_closed:
                # print " *", succ, next.effect
                forward_open.append(succ)
                forward_closed.add(succ)

    #extract relaxed plan:
    while goal:
        fact = goal.pop()
        if fact not in reached_by:
            continue
        next = reached_by[fact]
        actions.add((next.action, next.args))
        goal |= (next.preconds - facts)
        facts |= next.all_preconds
        
    # for a, args in actions:
    #     print "(%s %s)" % (a.name, " ".join(str(ar) for ar in args))

    return [a for a in actions if not a[0].name.startswith("axiom_")], facts
        

def initialize_ops(successors, stat):
    applicable_ops = []
    for ops in successors.itervalues():
        for o in ops:
            o.preconds = set(f for f in o.all_preconds if f not in stat)
            o.unsat_preconds = len(o.preconds)
            o.expanded = False
            o.initialised = False
            if not o.unsat_preconds:
                applicable_ops.append(o)
    return applicable_ops
    

def prob_explore(unary_successors, applicable_ops, stat, start_actions=[], prob_state=None):
    # unary_successors, applicable_ops = instantiate(actions, start, stat, domain, start_actions)
    
    def is_goal_action(op):
        for action, mapping in start_actions:
            if op.action.name == action.name:
                # print op, [mapping.get(a,a2).name for a,a2 in zip(action.args, op.args)], all(mapping.get(a,a2) == a2 for a,a2 in zip(action.args, op.args))
                if all(mapping.get(a,a2) == a2 for a,a2 in zip(action.args, op.args)):
                    return True
        return False

    relevant_prob_facts = set()

    def relaxed_reachable(init, successors, rand, full=False):
        random_vars = {}

        for ops in successors.itervalues():
            for o in ops:
                o.unsat_preconds = len(o.preconds)
                o.expanded = False
                o.initialised = False
        for op in init:
            op.expanded = False
            op.initialised = False

        goal_achievers = set()
        reachable = False
        open = init[:]
        reached_by = defaultdict(set)
        while open:
            next = open.pop(0)
            if full:
                if next.expanded:
                    continue
            else:
                if next.effect in reached_by:
                    continue
                
            next.expanded = True

            if is_goal_action(next):
                if not full:
                    return True
                goal_achievers.add(next)
                reachable = True

            # print next#, "=>", map(str, unary_successors[next.effect])
            expanded = next.effect in reached_by
            if next.effect:
                if next.effect.svar.modality == mapl.hyp:
                    svar = next.effect.svar.nonmodal()
                    val = next.effect.svar.modal_args[0]
                    relevant_prob_facts.add(state.Fact(svar, val))
                    if rand:
                        if svar in random_vars:
                            sval = random_vars[svar]
                        elif prob_state.is_det(svar):
                            sval = stat[svar]
                        else:
                            sval = prob_state[svar].sample(rand)
                            random_vars[svar] = sval
                        # print "random: %s = %s (expected: %s)" % (str(svar), str(sval), str(val))
                        if sval != val:
                            continue
                        
                reached_by[next.effect].add(next)
                # elif next.effect not in prob:
                #     prob[next.effect] = next.prob
                #     p_vars[next.effect] = next.p_vars
                # else:
                #     prob[next.effect] = 1 - (1-prob[next.effect]) * (1-next.prob)
                #     p_vars[next.effect] |= next.p_vars
                # print next.effect, "has prob.", prob[next.effect]
            if expanded:
                continue

            for succ in successors[next.effect]:
                if succ.expanded:
                    continue
                # if succ.action.name.startswith("look"):
                #     print "  ", succ, next.effect, succ.unsat_preconds 

                succ.unsat_preconds -= 1
                if succ.unsat_preconds == 0 and not succ.initialised: # and succ.effect not in forward_closed:
                    succ.initialised = True
                    # print " *", succ
                    open.append(succ)
                    # forward_closed.add(succ)

        def get_fact(prop):
            if prop.svar.modality == mapl.hyp:
                return state.Fact(prop.svar.nonmodal(), prop.svar.modal_args[0])
            return prop

        parents_left = {}
        prop_constraints = {}
        op_constraints = {}
        fluents = set()

        def find_parents(op, visited=set()):
            def find_prop(prop, vis):
                if prop not in parents_left:
                    parents_left[prop] = 1
                    new_vis = set([prop]) | vis
                    for op in reached_by[prop]:
                        find_parents(op, new_vis)
                else:
                    parents_left[prop] += 1
            if op.effect:
                fluents.add(op.effect.svar)
            for p in op.preconds:
                if p not in visited:
                    find_prop(p, visited)
                    

        def propagate_op_constraints(op, constraints, d=0):
            new_constraints = op_constraints.setdefault(op, dict())
            pre_constraints = ((f.svar, set([f.value])) for f in map(get_fact, op.preconds))
            # print "  "*d, "->", op
                
            for svar, vals in chain(constraints.iteritems(), pre_constraints):
                if svar in fluents:
                    continue
                # print "  "*d, svar
                if svar in new_constraints:
                    sub = new_constraints[svar] & vals
                    if not sub:
                        # print "  "*d, "  mutex:", svar, map(str, new_constraints[svar]), map(str, vals)
                        op_constraints[op] = None
                        return
                    new_constraints[svar] = sub
                else:
                    new_constraints[svar] = set(vals)
            for p in op.preconds:
                propagate_prop_constraints(p, new_constraints, d+1)
                
        def propagate_prop_constraints(prop, constraints, d=0):
            if parents_left[prop] == 0:
                # print "  "*d, "-> done", prop
                return
            # print "  "*d, "->", prop

            if prop not in prop_constraints:
                prop_constraints[prop] = constraints.copy()
            else:
                new_constraints = prop_constraints[prop]
                delete = []
                for svar, vals in new_constraints.iteritems():
                    if svar not in constraints:
                        delete.append(svar)
                    else:
                        new_constraints[svar] = vals | constraints[svar]
                        
                for svar in delete:
                    del new_constraints[svar]

            parents_left[prop] -= 1
            # print "  "*d, parents_left[prop], "left" 
            if parents_left[prop] == 0:
                for op in reached_by[prop]:
                    propagate_op_constraints(op, prop_constraints[prop], d+1)
                    
        if full:
            # print "-------------------"
            rel = defaultdict(set)
            rel_ops = set()
            for op in goal_achievers:
                find_parents(op)

            for op in goal_achievers:
                propagate_op_constraints(op, {})

            for op, con in op_constraints.iteritems():
                if con is not None:
                    # print
                    # print op
                    for svar, vals in con.iteritems():
                        rel[svar] |= vals
                        # print svar, map(str, vals)
            for svar, vals in rel.iteritems():
                print svar, map(str, vals)
            return reachable, rel
                        
        return reachable
    
    t0 = time.time()

    rand = random.Random()
    rand.seed()
    reachable, relevant_facts = relaxed_reachable(applicable_ops, unary_successors, None, True)
    if not reachable:
        print "NOT relaxed reachable!"
        return relevant_facts, 0.0
        
    reachable_succ = {}
    for prop, ops in unary_successors.iteritems():
        reachable_succ[prop] = []
        for op in ops:
            if op.initialised and (prop is None or prop in op.preconds):
                # if op.action.name.startswith("move"):
                #     print prop, "=>  ", op, len(op.preconds)
                reachable_succ[prop].append(op)
            # else:
            #     if op.action.name.startswith("move"):
            #         print prop, "!=> ", op

    # probs = []
    # for x in xrange(0,10):
    #     successes = 0
    #     for y in xrange(0,10):
    #         if relaxed_reachable(applicable_ops, reachable_succ, rand):
    #             successes += 1
    #     # if successes == 10:
    #     #     import pdb
    #     #     pdb.set_trace()
                
    #     probs.append(successes/10.0)


    # mean = sum(probs)/len(probs)
    # var = sum((p-mean)**2 for p in probs) / (len(probs)-1)


    # print "p_succ:", mean, var**0.5
    

    # print "total time for plangraph generation: %.2f" % (time.time()-t0)
        
    return relevant_facts,  1.0#mean + var**0.5

if __name__ == '__main__':
    import cProfile
    assert len(sys.argv) == 3, """Call 'planner.py domain.mapl task.mapl' for a single planner call"""
    domain_fn, problem_fn = sys.argv[1:]
    dom = pddl.load_domain(domain_fn)
    prob = pddl.load_problem(problem_fn, dom)
    
    t = pddl.mapl.MAPLObjectFluentNormalizer()
    dom = t.translate(dom)
    
    def goal_facts(cond):
        for lit in visitors.visit(cond, visitors.collect_literals, []):
            yield state.Fact.from_literal(lit)
        
    start = set(goal_facts(prob.goal))
    st = state.State.from_problem(prob)

    # cProfile.run('explore(dom.actions, start, st, dom)', 'mapsim_profile')
    explore(dom.actions, start, st, dom)
    
