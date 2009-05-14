# stdlib imports
import os.path
from string import Template

# mapsim imports
import config
from config import log
import simulation
import agent
import state
import utils

# gridworld domain imports
import graphical_problem
from constants_gridworld import *


class GridSimulationObject(simulation.SimulationObject):
    def build_mapl_problem(self, state=None, goal=None, sfacts=None):
        if state is None:
            state = self.world_state
        if goal is None:
            goal = self.goal
        if sfacts is None:
            sfacts = self.static_facts
        additional_facts = []
        # associate objects with types
        objs = self.find_known_objects()
        #print "known objects", objs
        pref2type = PREF2TYPE_ALL_AGENTS if isinstance(self, simulation.Simulation) else PREF2TYPE_ONE_AGENT
        obj_def = [(obj, guess_type(obj, pref2type)) for obj in objs]
        agts = [o for (o,t) in obj_def if t == "agent" or o == self.name]
        obj_def = ["%s - %s" % (o, t) for o, t in obj_def if t]
        obj_def.append("%s - planning_agent" % self.name)
        # find facts
        #facts = ["(%s : %s)" % (str(k), state[k]) for k in state]
        facts = [state.mapl_assignment(svar) for svar in state]
        # add activation info
        use_time_outs = False
        if use_time_outs:
            timeout = 3
            for n in range(timeout):
                obj_def.append("n%d - number" % (n+1))   # n0 should already be there
                facts.append("(__succ n%d : n%d)" % (n,n+1))
            for a in agts:
                t = 0 #if a == self.name else timeout
                facts.append("(__timeout %s : n%d)" % (a, t))
                facts.append("(__timeout_max %s : n%d)" % (a, t))
        # find goalfacts
        #gfacts = ["(%s : %s)" % (str(k), goal[k]) for k in goal]
        gfacts = [goal.mapl_assignment(svar) for svar in goal]
        # join all these to build the MAPL problem definition
        subs = dict(obj_def=obj_def, facts=facts, goal=gfacts, static_facts=sfacts, additional_facts=additional_facts)
        templ = Template(PROB_TEMPLATE)
        indent = "  "
        for k in subs:
            subs[k] = ["%s%s" % (indent, line) for line in subs[k]]
            subs[k] = "\n".join(subs[k])
        problem = templ.safe_substitute(subs)
        # for the time being we want to create PDDL problems
        # instead of MAPL ones.  What follows is a quick hack
        # to achieve this (works only for simple problems like
        # the ones in this domain)
        problem = problem.replace(" : ", " ")
        return problem


class GridAgent(agent.Agent, GridSimulationObject):
    pass

class Simulation(simulation.Simulation, GridSimulationObject):

    def load_scenario(self):
        config_keys = "grid_size num_agents num_obstacles sensor_range " \
        "memory_span share_goals cooperation hierarchy commands plans " \
        "solvable solved time planning_time timeouts plans_kept".split()
        self.register_stats(config_keys)
        fn = os.path.join(config.scenario_path)
        scen = graphical_problem.load(fn)
        init, goal = [list(tuples2sv_assignments(tup)) for tup in (scen.itups, scen.gtups)]
        self.world_state.update(init)
        self.goal.update(goal)
        self.grid_size = scen.size
        self.static_facts = make_static_facts(self.grid_size, self.sensor_range)
        self.find_known_objects()
        self.num_agents = 0
        self.num_obstacles = 0
        for svar, agt_name in goal:
            if agt_name.startswith("agt"):
                agent = GridAgent(self, name=agt_name)
                self.num_agents += 1
                agent.goal[svar] = agt_name
                agent.static_facts = self.static_facts
                agent.known_objects = set(self.known_objects)
            elif agt_name == "obstacle":
                self.num_obstacles += 1
        
    def show_current_state(self):
        size = self.grid_size
        current = graphical_problem.state2graphics(self.world_state, size)
        goal = graphical_problem.state2graphics(self.goal, size)
        both = utils.strlists2columns((current, goal))
        log(both, screen=True)

    def unique_string_for_setting(self, setting_dict):
        s = "m%(memory_duration)s:s%(sensor_range)s" % setting_dict
        return s

def tuples2sv_assignments(tups):
    for obj, row, col in tups:
        svar = state.StateVariable((OCC, "c%d%d" % (row, col)))  # TODO: predicate missing!!!
        val = obj
        yield svar, val

def sdist(t1, t2):
    r1, c1 = t1
    r2, c2 = t2
    return max(abs(r1-r2), abs(c1-c2))

def mhdist(t1, t2):
    r1, c1 = t1
    r2, c2 = t2
    return abs(r1-r2) + abs(c1-c2)

def make_static_facts(size, sensor_range):
    tuples = [(r,c) for r in xrange(size) for c in xrange(size)]
    tup2cell = {}
    for (r,c) in tuples:
        tup2cell[(r,c)] = "c%d%d" % (r,c)
    cartesian__tups = [(t1,t2) for t1 in tuples for t2 in tuples]
    conn_facts = []
    distance_facts = []
    for t1, t2 in cartesian__tups:
        mh = mhdist(t1, t2)
        if mh == 1:
            conn_facts.append("(connected %s %s)" % (tup2cell[t1], tup2cell[t2]))
    if sensor_range:
        for t1, t2 in cartesian__tups:
            sd = sdist(t1, t2)
            if sd <= sensor_range:
                distance_facts.append("(in-sensing-distance %s %s)" % (tup2cell[t1], tup2cell[t2]))
    return conn_facts + distance_facts

def guess_type(obj, pref2type, debug=False):
    assert utils.is_string(obj)
    for prefix in pref2type:
        if debug: print prefix
        if obj[:len(prefix)] == prefix:  # just to test whether this is faster than .startswith()
            return pref2type[prefix]
        elif debug:
            print prefix, "!=", obj
    if not debug:
        guess_type(obj, pref2type, True)
    raise Exception("Cannot guess MAPL type of object '%s'." % obj)

