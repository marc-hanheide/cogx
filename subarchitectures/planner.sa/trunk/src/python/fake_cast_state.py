import re
import time

from itertools import chain
from standalone import config, pddl
from standalone.pddl import prob_state
import standalone.globals as global_vars

import cast_state

from de.dfki.lt.tr.beliefs.slice import logicalcontent
import cast.cdl

log = config.logger("PythonServer")
BINDER_SA = "binder"
#CAST_OBJ_RE = re.compile("[a-z]+_(_?[0-9a-z])_(_?[0-9a-z])")

class FakeCASTState(cast_state.CASTState):
    def __init__(self, problem, domain, component=None, consistency_cond=None):
        t0 = time.time()
        self.config = global_vars.config
        self.domain = domain
        self.problem = problem
        self.consistency_cond = consistency_cond
      #self.beliefs = beliefs
        self.beliefdict = {}
        self.address_dict = {}
  
        self.objects = set(problem.objects)
        self.castname_to_obj = {}
        self.obj_to_castname = {}

        self.prob_state = prob_state.ProbabilisticState.from_problem(problem)
        log.debug("time to state generation: %.2f", time.time() - t0)
        
        self.raw_state = prob_state.ProbabilisticState.from_problem(problem)
        self.raw_objects = set(problem.objects)
        log.debug("time to 2nd state generation: %.2f", time.time() - t0)

        if component:
            coma_facts, coma_objects = self.get_coma_data(component)
            default_facts, default_objects = self.get_default_data(component)
            for o in chain(coma_objects, default_objects):
                if o not in problem.objects:
                    self.objects.add(o)
                    problem.add_object(o)
                    
            for f in chain(coma_facts, default_facts):
                self.prob_state.set(f)
                problem.init.append(f.as_literal(useEqual=True))

        # import debug
        # debug.set_trace()
        self.generated_facts, self.generated_objects = self.generate_init_facts(problem, None)
        log.debug("time to init fact generation: %.2f", time.time() - t0)
        for o in self.generated_objects:
            if o not in self.objects:
                self.objects.add(o)
        for f in self.generated_facts:
            self.prob_state.set(f)
            problem.init.append(f.as_literal(useEqual=True))

        # print map(str, self.objects)
            
        # print "objects:",  map(str,self.objects)
        self.state = self.prob_state.determinized_state(0.05, self.config.uncertainty_threshold)
        log.debug("time to state determinisation: %.2f", time.time() - t0)

        self.consistent = self.check_consistency(self.state)
        log.debug("time to consistency check: %.2f", time.time() - t0)
        
        self.generate_belief_state(self.prob_state, self.state)
        log.debug("time to belief state generation: %.2f", time.time() - t0)
        
        commit_facts = self.generate_committed_facts(self.state)
        for f in commit_facts:
            self.state.set(f)
        
    def convert_percepts(self, percepts):
        return []

    def featvalue_from_object(self, arg):
        if arg not in self.obj_to_castname and not arg.is_instance_of(pddl.t_number):
            obj_match = cast_state.CAST_OBJ_RE.search(arg.name)
            if obj_match:
                i1 = obj_match.group(1)
                i2 = obj_match.group(2)
                if len(i1) == 2:
                    i1 = i1[1].upper()
                if len(i2) == 2:
                    i2 = i2[1].upper()
                name = "%s:%s" % (i1, i2)
                self.obj_to_castname[arg] = name
                self.address_dict[name] = cast.cdl.WorkingMemoryAddress(name, "fake.sa")
            # else:
            #     vo_match = VIRTUAL_OBJ_RE.search(arg.name)
            #     if vo_match and vo_match.group(1) in self.domain.types:

        return cast_state.CASTState.featvalue_from_object(self, arg)
    
    def update_beliefs(self, diffstate):
        return []
