from itertools import chain
from standalone import config
from standalone.pddl import prob_state

import cast_state

from de.dfki.lt.tr.beliefs.slice import logicalcontent

log = config.logger("PythonServer")
BINDER_SA = "binder"

class FakeCASTState(cast_state.CASTState):
    def __init__(self, problem, domain, component=None):
        self.domain = domain
        self.problem = problem
        #self.beliefs = beliefs
        #self.beliefdict = dict((b.id, b) for b in beliefs)
        #TODO: make this less ugly
        #tp.current_domain = self.domain
        #tp.belief_dict = self.beliefdict
  
        #obj_descriptions = list(tp.unify_objects(tp.filter_unknown_preds(tp.gen_fact_tuples(beliefs))))
  
        self.objects = set(problem.objects)
        self.namedict = {}

        self.prob_state = prob_state.ProbabilisticState.from_problem(problem)

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
        # print map(str,self.generated_objects)
        for o in self.generated_objects:
            if o not in problem.objects:
                self.objects.add(o)
                problem.add_object(o)
        for f in self.generated_facts:
            self.prob_state.set(f)
            problem.init.append(f.as_literal(useEqual=True))
            
        self.state = self.prob_state.determinized_state(0.05, 0.95)

        self.generate_belief_state(self.prob_state, self.state)
        
        commit_facts = self.generate_committed_facts(self.state)
        for f in commit_facts:
            self.state.set(f)
        
    def convert_percepts(self, percepts):
        return []

    def featvalue_from_object(self, arg):
        #arg is a domain constant
        name = arg.name
        value = logicalcontent.ElementaryFormula(0, name)

        return value
    
    def update_beliefs(self, diffstate):
        return []
