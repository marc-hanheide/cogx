from standalone import pddl, config
from standalone.pddl import state, prob_state

import cast_state

import de.dfki.lt.tr.beliefs.slice as bm
from de.dfki.lt.tr.beliefs.slice import logicalcontent, distribs
import cast.cdl

import task_preprocessor as tp

log = config.logger("PythonServer")
BINDER_SA = "binder"

class FakeCASTState(cast_state.CASTState):
    def __init__(self, problem, domain):
        self.domain = domain
        self.problem = problem
        #self.beliefs = beliefs
        #self.beliefdict = dict((b.id, b) for b in beliefs)
        #TODO: make this less ugly
        #tp.current_domain = self.domain
        #tp.belief_dict = self.beliefdict
  
        #obj_descriptions = list(tp.unify_objects(tp.filter_unknown_preds(tp.gen_fact_tuples(beliefs))))
  
        self.objects = problem.objects
        self.namedict = {}
        
        self.prob_state = prob_state.ProbabilisticState.from_problem(problem)
        self.state = self.prob_state.determinized_state(0.05, 0.95)

    def convert_percepts(self, percepts):
        return []

    def featvalue_from_object(self, arg):
        #arg is a domain constant
        name = arg.name
        value = logicalcontent.ElementaryFormula(0, name)

        return value
    
    def update_beliefs(self, diffstate):
        return []
