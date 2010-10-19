from itertools import chain
import re

from standalone import pddl, config
from standalone.pddl import state, prob_state

import de.dfki.lt.tr.beliefs.slice as bm
from de.dfki.lt.tr.beliefs.slice import logicalcontent, distribs
import eu.cogx.beliefs.slice as eubm
import cast.cdl

import task_preprocessor as tp

log = config.logger("cast-state")
BINDER_SA = "binder"
TOTAL_P_COSTS = 200

QDL_NUMBER_RE = re.compile("\"([-eE0-9\.]+)\"\^\^<xsd:float>")
QDL_VALUE = re.compile("<dora:(.*)>")
#QDL_VALUE = re.compile("<http://dora.cogx.eu#(.*)>")

class CASTState(object):
    def __init__(self, beliefs, domain, oldstate=None, component=None):
        self.domain = domain
        self.beliefs = []
        for b in beliefs:
            if isinstance(b, eubm.GroundedBelief) or isinstance(b, eubm.AssertedBelief):
                self.beliefs.append(b)

        self.beliefdict = dict((b.id, b) for b in self.beliefs)

        self.address_dict = component.address_dict if component else None
        
        self.coma_facts = []
        self.coma_objects = set()
        if oldstate and oldstate.coma_facts is not None:
            self.coma_facts = oldstate.coma_facts
            self.coma_objects = oldstate.coma_objects
        elif component:
            self.coma_facts, self.coma_objects = self.get_coma_data(component)
            
        #TODO: make this less ugly
        tp.current_domain = self.domain
        tp.belief_dict = self.beliefdict

        obj_descriptions = list(tp.unify_objects(tp.filter_unknown_preds(tp.gen_fact_tuples(self.beliefs))))
  
        #keep names for previously matched objects
        renamings = {}
        if oldstate:
            for gen in oldstate.generated_objects:
                if gen.name in oldstate.namedict:
                    belname = oldstate.namedict[gen.name]
                    renamings[belname] = gen.name
                    #print "previous match:", belname, gen.name

        objects = tp.infer_types(obj_descriptions)

        self.namedict = tp.rename_objects(objects, self.coma_objects|domain.constants, add_renamings=renamings )
        self.objects = set(list(objects)) # force rehashing
        self.facts = list(tp.tuples2facts(obj_descriptions))
        self.objects |= self.coma_objects
        self.facts += self.coma_facts
            
        problem = pddl.Problem("cogxtask", self.objects, [], None, domain)
        self.prob_state = prob_state.ProbabilisticState(self.facts, problem)

        self.generated_facts, self.generated_objects = self.generate_init_facts(problem, oldstate)
        self.facts += self.generated_facts
        for f in self.generated_facts:
            self.prob_state.set(f)
        self.objects |= self.generated_objects

        self.state = self.prob_state.determinized_state(0.05, 0.95)
        if oldstate:
            self.match_generated_objects(oldstate)

            
    def generate_init_facts(self, problem, oldstate=None):
        cstate = self.prob_state.determinized_state(0.05, 0.95)

        generated_facts = {}
        generated_objects = set()
        new_objects = set(self.objects)
        if oldstate and oldstate.generated_facts is not None:
            current_objects = self.objects | oldstate.generated_objects | self.domain.constants
            
            for gen in oldstate.generated_objects:
                if gen.name in oldstate.namedict and gen.name not in self.namedict:
                    log.debug("generated object %s is gone", gen.name)
                    current_objects.discard(gen) # remove matched generated objects that have vanished
                    
            for svar, val in oldstate.generated_facts:
                if all(a in current_objects or a.is_instance_of(pddl.t_number) for a in chain(svar.args, svar.modal_args, [val])):
                    generated_facts[svar] = val

            cstate.update(generated_facts)
            generated_objects = oldstate.generated_objects & current_objects
            new_objects -= oldstate.objects
            for o in generated_objects:
                problem.add_object(o)

        # import debug
        # debug.set_trace()
        for rule in self.domain.init_rules:
            def inst_func(mapping, args):
                if len(args) != len(rule.args):
                    return True, None
                elif len(rule.args) == 0 and oldstate is None:
                    return True, None
                #elif rule.precondition and any(a in new_objects for a in mapping.itervalues()):
                #    return True, None
                return True, None
            
            for mapping in rule.smart_instantiate(inst_func, rule.args, [problem.get_all_objects(a.type) for a in rule.args], problem):
                if rule.precondition is None or cstate.is_satisfied(rule.precondition):
                    for e in rule.get_effects():
                        #print e.pddl_str()
                          #apply effects seperately in order to allow later effects affecting previously set values
                        facts = cstate.get_effect_facts(e)
                        #print ["%s=%s" % (str(k), str(v)) for k,v in facts.iteritems()]
                        cstate.update(facts)
                        generated_facts.update(facts)

        generated_objects |= (problem.objects - self.objects)
                      
        if 'partial-observability' in self.domain.requirements:
            generated_facts.update(self.compute_logps(cstate))
                      
        return [pddl.state.Fact(svar,val) for svar, val in generated_facts.iteritems()], generated_objects

    def compute_logps(self, st):
        import math
        pfuncs = set(f for f in self.domain.functions if "log-%s" % f.name in self.domain.functions)
        facts = []
        for svar, val in st.iteritems():
            if svar.function in pfuncs and val != pddl.UNKNOWN and val.is_instance_of(pddl.t_number) and val.value > 0:
                logfunc = self.domain.functions.get("log-%s" % svar.function.name, svar.args)
                logvar = pddl.state.StateVariable(logfunc, svar.args)
                logval = pddl.types.TypedNumber(max(-math.log(val.value, 2), 0.1))
                facts.append((logvar, logval))
        return facts
    
    def get_coma_data(self, component):
        coma_objects = set()
        coma_facts = []
        for f in chain(self.domain.predicates, self.domain.functions):
            if not f.name.startswith("dora__"):
                continue
            assert len(f.args) == 2
            query = "SELECT ?x ?y ?p where ?x <%s> ?y ?p" % f.name.replace("dora__","dora:")
            try:
                results = component.getHFC().querySelect(query)
            except Exception, e:
                log.warning("Error when calling the HFC server: %s", str(e))
                return coma_facts, coma_objects
            
            for elem in results.bt:
                a1_str = QDL_VALUE.search(elem[results.varPosMap['?x']]).group(1)
                a2_str = QDL_VALUE.search(elem[results.varPosMap['?y']]).group(1)
                p_str = QDL_NUMBER_RE.search(elem[results.varPosMap['?p']]).group(1)
                a1 = pddl.TypedObject(a1_str, f.args[0].type)
                a2 = pddl.TypedObject(a2_str, f.args[1].type)
                p = float(p_str)
                
                for a in (a1,a2):
                    if a not in self.domain.constants:
                        coma_objects.add(a)
                if f.type.equal_or_subtype_of(pddl.t_number):
                    val = pddl.types.TypedNumber(p)
                elif f.type == pddl.t_boolean:
                    if p > 0.0:
                        val = pddl.TRUE
                    else:
                        val = pddl.FALSE
                else:
                    assert False
                    
                fact = state.Fact(state.StateVariable(f, [a1, a2]), val)
                coma_facts.append(fact)
                #log.debug("added fact: %s", fact)
        return coma_facts, coma_objects

    def match_generated_objects(self, oldstate):
        # This will only match single new objects (i.e. no several new
        # objects referring to each other), but that should be enough
        # for now.
        def repl(a):
            if a == gen:
                return obj
            return a
        
        new_objects = self.objects - self.generated_objects - oldstate.objects
        matches = {}
        for gen in oldstate.generated_objects & self.objects:
            if gen.name in oldstate.namedict:
                log.debug("Keeping match from %s to %s.", gen.name, oldstate.namedict[gen.name])
                continue
            
            facts = []
            for f in oldstate.state.iterfacts():
                if any(a == gen for a in chain(f.svar.args, f.svar.modal_args)):
                    facts.append(f)
                
            for obj in new_objects:
                if obj in matches or obj.type != gen.type:
                    continue
                match = True
                for svar, val in facts:
                    if svar.function.name in ("is-virtual", ) or svar.modality in (pddl.mapl.indomain, pddl.mapl.i_indomain, pddl.mapl.knowledge, pddl.mapl.direct_knowledge):
                        continue # HACK: don't match on modal/virtual predicates
                    newvar = pddl.state.StateVariable(svar.function, [repl(a) for a in svar.args], svar.modality,  [repl(a) for a in svar.modal_args])
                    if self.state[newvar] == pddl.UNKNOWN and val == pddl.UNKNOWN:
                        continue
                    if self.state[newvar] != val:
                        log.debug("Mismatch: %s = %s != %s = %s", str(svar), val.name, str(newvar), self.state[newvar].name)
                        match = False
                        break
                if match:
                    matches[obj] = gen
                    break

        if matches:
            matched = matches.values()
            #remove old (generated) facts
            facts_to_remove = set(f.svar for f in self.facts if any(a in matched for a in chain(f.svar.args, f.svar.modal_args, [f.value])))
            self.facts = [f for f in self.facts if f.svar not in facts_to_remove]
            self.generated_facts = [f for f in self.generated_facts if f.svar not in facts_to_remove]
            
            for obj, gen in matches.iteritems():
                self.objects.discard(obj)
                log.debug("Matching generated object %s to new object %s.", gen.name, obj.name)
                belname = self.namedict[obj.name]
                del self.namedict[obj.name]
                del self.namedict[belname]
                
                obj.rename(gen.name)
                self.namedict[belname] = obj
                self.namedict[obj.name] = belname

            for f in self.facts:
                f.svar.rehash()

            problem = pddl.Problem("cogxtask", self.objects, [], None, self.domain)
            self.prob_state = prob_state.ProbabilisticState(self.facts, problem)
            #self.prob_state.apply_init_rules(domain = self.domain)
            #self.generated_objects = set(problem.objects) - self.objects
            self.state = self.prob_state.determinized_state(0.05, 0.95)


    def to_problem(self, slice_goals, deterministic=True, domain=None):
        if "action-costs" in self.domain.requirements:
            opt = "minimize"
            opt_func = pddl.FunctionTerm(pddl.builtin.total_cost, [])
        else:
            opt = None
            opt_func = None

        if domain is None:
            domain = self.domain

        if deterministic:
            facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in self.state.iterfacts()]
            if 'numeric-fluents' in domain.requirements:
                pass
                #b = pddl.Builder(domain)
                #facts.append(b.init('=', (pddl.dtpddl.total_p_cost,), TOTAL_P_COSTS))
        else:
            facts = self.facts

        problem = pddl.Problem("cogxtask", self.objects | self.generated_objects, facts, None, domain, opt, opt_func )

        if deterministic:
            self.state.problem = problem
        else:
            self.prob_state.problem = problem

        if slice_goals is None:
            return problem, None

        goaldict = {}
        problem.goal = pddl.conditions.Conjunction([], problem)
        for goal in slice_goals:
            try:
                goalstrings = tp.transform_goal_string(goal.goalString, self.namedict).split("\n")
                pddl_goal = pddl.parser.Parser.parse_as(goalstrings, pddl.conditions.Condition, problem)
            except pddl.parser.ParseError,e:
                log.warning("Could not parse goal: %s", goal.goalString)
                log.warning("Error: %s", e.message)
                continue
            
            goaldict[pddl_goal] = goal
            goaldict[goal.goalString] = pddl_goal
            
            if goal.importance < 0:
                problem.goal.parts.append(pddl_goal)
            else:
                problem.goal.parts.append(pddl.conditions.PreferenceCondition(goal.importance, pddl_goal, problem))

        log.debug("goal: %s", problem.goal)
        
        return problem, goaldict

    def convert_percepts(self, percepts):
        objdict = dict((o.name, o) for o in chain(self.objects, self.domain.constants))
        tp.belief_dict = {}
        tp.belief_dict.update(self.beliefdict)
        percept2bel = {}

        for b in self.beliefs:
            tp.belief_dict[b.id] = b
            if isinstance(b.hist, bm.history.CASTBeliefHistory):
                for wmp in b.hist.ancestors:
                    wma = wmp.address
                    tp.belief_dict[wma.id] = b
                    percept2bel[wma.id] = b

        obj_descriptions = list(tp.unify_objects(tp.filter_unknown_preds(tp.gen_fact_tuples(percepts))))
        p_objects = tp.infer_types(obj_descriptions)
        facts = list(tp.tuples2facts(obj_descriptions))

        def replace_object(obj):
            if obj.name in objdict:
                return objdict[obj.name]
            elif obj.name in self.namedict:
                return self.namedict[obj.name]
            elif obj.name in percept2bel:
                bel = percept2bel[obj.name]
                return self.namedict[bel.id]
            else:
                log.warning("Percept %s has no matching grounded belief.", obj.name)
                return None 

        # import debug
        # debug.set_trace()
        
        filtered_facts = []
        for svar, value in facts:
            assert svar.modality is None
            n_args = [replace_object(a) for a in svar.args]
            
            if isinstance(value, pddl.prob_state.ValueDistribution):
                # only return the percept with the highest probability > 0
                highest = None
                for val, p in value.iteritems():
                    if p > value.get(highest, 0.0):
                        highest = val
                if highest is None:
                    continue
                value = highest
                    
            nval = replace_object(value)
            if all(a is not None for a in n_args) and nval is not None:
                fact = state.Fact(state.StateVariable(svar.function, n_args), nval)
                filtered_facts.append(fact)
                
        return filtered_facts

    def featvalue_from_object(self, arg):
        if arg.name in self.namedict:
            #arg is provided by the binder
            name = self.namedict[arg.name]
        else:
            #arg is a domain constant
            name = arg.name

        if name in self.beliefdict:
            #arg is a pointer to another belief
            wma = self.address_dict[name] if self.address_dict else cast.cdl.WorkingMemoryAddress(name, BINDER_SA)
            value = logicalcontent.PointerFormula(0, wma)
        elif arg.is_instance_of(pddl.t_boolean):
            value = False
            if arg == pddl.TRUE:
                value = True
            value = logicalcontent.BooleanFormula(0, value)
        elif arg == pddl.UNKNOWN:
            value = logicalcontent.UnknownFormula(0)
        else:
            if ":" in name:
                log.warning("%s seems to be a belief but is not in belief dict!", name)
                wma = self.address_dict[name] if self.address_dict else cast.cdl.WorkingMemoryAddress(name, BINDER_SA)
                value = logicalcontent.PointerFormula(0, wma)
            else:
                value = logicalcontent.ElementaryFormula(0, name)
            #assume a string value
            #value = featurecontent.StringValue(name)

        return value
    
    def update_beliefs(self, diffstate):
        changed_ids = set()
        new_beliefs = []

        def get_value_dist(dist, feature):
            #if isinstance(dist, distribs.DistributionWithExistDep):
            #    return get_feature_dist(dist.Pc, feature)
            if isinstance(dist, distribs.CondIndependentDistribs):
                if feature in dist.distribs:
                    return dist.distribs[feature].values, dist
                return None, dist
            if isinstance(dist, distribs.BasicProbDistribution):
                if dist.key == feature:
                    return dist.values, None
                return None, None
            assert False, "class %s not supported" % str(type(dist))

        def find_relation(args):
            result = None

            arg_values = [self.featvalue_from_object(arg) for arg in args]

            for bel in self.beliefs:
                if not bel.type == "relation":
                    continue

                found = True
                for i, arg in enumerate(arg_values):
                    dist, _ = get_value_dist(bel.content, "element%d" % i)
                    if not dist:
                        found = False
                        break

                    elem = dist.values[0].val
                    if elem != arg:
                        found = False
                        break

                if found:
                    result = bel
                    break

            if result:
                return result

            frame = bm.framing.SpatioTemporalFrame()
            eps = bm.epstatus.PrivateEpistemicStatus("robot")
            hist = bm.history.CASTBeliefHistory([], [])
            dist_dict = {}
            for i, arg in enumerate(arg_values):
                feat = "element%d" % i
                pair = distribs.FeatureValueProbPair(arg, 1.0)
                dist_dict[feat] = distribs.BasicProbDistribution(feat, distribs.FeatureValues([pair])) 
            dist = distribs.CondIndependentDistribs(dist_dict)
            result = bm.sitbeliefs.dBelief(frame, eps, "temporary", "relation", dist, hist)
            return result

        for svar, val in diffstate.iteritems():
            if len(svar.args) == 1:
                obj = svar.args[0]
                try:
                    bel = self.beliefdict[self.namedict[obj.name]]
                except:
                    log.warning("tried to find belief for %s, but failed", str(obj))
                    continue
            else:
                bel = find_relation(svar.args)
                if bel.id == "temporary":
                    self.beliefs.append(bel)
                    new_beliefs.append(bel)

            feature = svar.function.name
            dist, parent = get_value_dist(bel.content, feature)
            if not dist:
                if isinstance(parent, distribs.CondIndependentDistribs):
                    dist = distribs.FeatureValues()
                    parent.distribs[feature] = distribs.BasicProbDistribution(feature, dist)
                else:
                    continue

            #TODO: deterministic state update for now
            if isinstance(dist, distribs.NormalValues):
                dist.mean = val.value
                dist.variance = 0;
            else:
                fval = self.featvalue_from_object(val)
                pair = distribs.FeatureValueProbPair(fval, 1.0)
                dist.values = [pair]

            if bel.id != "temporary":
                changed_ids.add(bel.id)

        return [self.beliefdict[id] for id in changed_ids] + new_beliefs
    
