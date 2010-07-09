from standalone import pddl, config
from standalone.pddl import state, prob_state

import de.dfki.lt.tr.beliefs.slice as bm
from de.dfki.lt.tr.beliefs.slice import logicalcontent, distribs
import cast.cdl

import task_preprocessor as tp

log = config.logger("PythonServer")
BINDER_SA = "binder"

class CASTState(object):
    def __init__(self, beliefs, domain):
        self.domain = domain
        self.beliefs = beliefs
        self.beliefdict = dict((b.id, b) for b in beliefs)
        print self.beliefdict.keys()
        #TODO: make this less ugly
        tp.current_domain = self.domain
        tp.belief_dict = self.beliefdict
  
        obj_descriptions = list(tp.unify_objects(tp.filter_unknown_preds(tp.gen_fact_tuples(beliefs))))
  
        self.objects = tp.infer_types(obj_descriptions)
        self.namedict = tp.rename_objects(self.objects)

        self.facts = list(tp.tuples2facts(obj_descriptions))
        self.prob_state = prob_state.ProbabilisticState(self.facts, None)
        self.state = self.prob_state.determinized_state(0.1, 0.9)

    def to_problem(self, cast_task, deterministic=True):
        if "action-costs" in self.domain.requirements:
            opt = "minimize"
            opt_func = pddl.FunctionTerm(pddl.builtin.total_cost, [])
        else:
            opt = None
            opt_func = None

        if deterministic:
            facts = [f.as_literal(useEqual=True, _class=pddl.conditions.LiteralCondition) for f in self.state.iterfacts()]
        else:
            facts = self.facts

        problem = pddl.Problem("cogxtask", self.objects, facts, None, self.domain, opt, opt_func )

        if cast_task is None:
            return problem

        problem.goal = pddl.conditions.Conjunction([], problem)
        for goal in cast_task.goals:
            goalstrings = tp.transform_goal_string(goal.goalString, self.namedict).split("\n")
            pddl_goal = pddl.parser.Parser.parse_as(goalstrings, pddl.conditions.Condition, problem)
            if goal.importance < 0:
                problem.goal.parts.append(pddl_goal)
            else:
                problem.goal.parts.append(pddl.conditions.PreferenceCondition(goal.importance, pddl_goal, problem))

        log.debug("goal: %s", problem.goal)
        self.state.problem = problem
        
        return problem
      

    def featvalue_from_object(self, arg):
        if arg in self.namedict:
            #arg is provided by the binder
            name = self.namedict[arg]
        else:
            #arg is a domain constant
            name = arg.name

        if name in self.beliefdict:
            #arg is a pointer to another belief
            value = logicalcontent.PointerFormula(0, cast.cdl.WorkingMemoryAddress(name, BINDER_SA))
        elif arg.is_instance_of(pddl.t_boolean):
            value = False
            if arg == pddl.TRUE:
                value = True
            value = logicalcontent.BooleanFormula(0, value)
        elif arg == pddl.UNKNOWN:
            value = logicalcontent.UnknownFormula(0)
        else:
            if ":" in name:
                value = logicalcontent.PointerFormula(0, cast.cdl.WorkingMemoryAddress(name, BINDER_SA))
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
                    bel = self.beliefdict[self.namedict[obj]]
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
    
