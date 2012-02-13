#! /usr/bin/env python
# -*- coding: latin-1 -*-

import os.path
import sys

import state
import actions
import axioms
import sensors
import conditions
import predicates
import parser
import types
import effects
import utils
import config
from constants import *
import itertools

WRITE_INITIALLY_FACTS = False
WRITE_COMPLEX_INITIALLY_FACTS = False
WRITE_IN_DOMAIN_PRED = False

def problem2mapl_str(problem):
    def gen():
        # fact file: preamble
        yield "(define (problem %s)" % problem.task_name
        yield "(:domain %s)" % problem.domain_name
        #objects
        yield "(:objects"
        for obj in problem.objects:
            yield "  %s" % types.pddl_str(obj)
        yield ")"
        #init

        yield "(:init"
#         for svar, val in problem.init_state.items():
#             if svar.name == "=":
#                 continue
#             factstr = state.str_assignment(svar, val, add_parens=True)
#             yield factstr
#             if WRITE_INITIALLY_FACTS:
#                 if WRITE_COMPLEX_INITIALLY_FACTS or not (factstr.startswith("k__") or factstr.startswith("d__")):
#                     if not (factstr.startswith("k__") or factstr.startswith("d__")):
#                         yield factstr.replace(svar.name, "i__"+self.predicate, 1)
        for fact in problem.init_state.get_facts():
            yield fact.as_mapl_str()
        yield ")"

        yield "(:goal " 
        yield problem.goal.pddl_str()
        yield "))"
    return "\n".join(gen())


def problem2pddl_str(problem):
    def gen():
        # fact file: preamble
        yield ";; generated by MAPL2PDDL" 
        yield "(define (problem %s)" % problem.task_name
        yield "(:domain %s)" % problem.domain_name
        #objects
        yield "(:objects"
        for obj in problem.objects:
            yield "  %s" % types.pddl_str(obj)
        yield ")"
        #init

        yield "(:init"
        for fact in problem.init_conditions:
            if fact.predicate == "=":
                continue
            if fact.is_negated():
                continue
            #yield fact.pddl_str("  ")
            factstr = fact._dump_pddl(use_direct_k=True)
            yield "%s%s" % ("  ", factstr)
            if WRITE_INITIALLY_FACTS:
                if WRITE_COMPLEX_INITIALLY_FACTS or not (factstr.startswith("k__") or factstr.startswith("d__")):
                    if not (factstr.startswith("k__") or factstr.startswith("d__")):
                        yield fact.pddl_str_initially("  ")
        yield ")"


        yield "(:goal " 
        yield problem.goal.pddl_str()
        yield "))"
    return "\n".join(gen())


class Problem(object):
    """ currently only a little placeholder.  to be beefed up ASAP"""
#     def __init__(self, task, mapl_domain):
#         task_name, domain_name, objects, init, goal = task
    def __init__(self, task_name, domain_name, objects, init, goal):
        self.task_name = task_name
        self.domain_name = domain_name
        self.objects = objects
        self.task_objects = list(objects)  #objects only appearing in this task
        self.init_conditions = init
        self.init_state = state.State(facts=(atom.as_svar_assignment() for atom in init))
        self.goal = goal
        self.mapl_domain = None
        self.grounding_state = None  # only used by server_implementation_cosy.py right now
    
    def goal_objects(self, goal_str):
        goal_str = goal_str.replace("("," ").replace(")"," ")
        goal_elmts = set(goal_str.split())
        obj_set = set(o.name for o in self.objects)
#         print "ge: %s" % repr(goal_elmts)
#         print "os: %s" % repr(obj_set)
        goal_objects = [o for o in self.objects if o.name in goal_elmts]
        return goal_objects

    def as_data_list(self):
        """ TODO: ugly refactoring artefact. should disappear ASAP """
        return self.task_name, self.domain_name, self.objects, self.init_conditions, self.goal

    def to_pddl_str(self):
        s = problem2pddl_str(self)
        return s

    def to_mapl_str(self):
        s = problem2mapl_str(self)
        return s



class Task(object):
    def __init__(self, domain_name, requirements, the_types, objects, predicates, actions, axioms, planning_agent=None, sensors=None):
        self.domain_name = domain_name
        self.requirements = requirements
        self.objects = objects
        self.constants = list(objects)
        self.types = the_types
        self.type_tree = types.TypeTree.from_typed_list(the_types)
        self.predicates = dict((p.name,p) for p in predicates)
        #self.update_predicate_lists()
        self.actions = actions
        self.pddl_actions = None
        self.name2actions = None
        self.axioms = axioms
        self.sensors = sensors
        self.sname2sensor = dict((s.name, s) for s in sensors)
        self.planning_agent = planning_agent
        self.axiom_counter = 0
        

#     def set_problem_data(self, problem):
#         # TODO: ugly mid-refactoring stuff.  The problem should reside in a
#         # independent class ASAP
#         task_name, domain_name, objects, init, goal = problem.as_data_list()
#         self.task_name = task_name
#         self.objects += objects
#         self.task_objects = list(objects)  #objects only appearing in this task
#         self.init = init
#         self.goal = goal
    
    @staticmethod
    def parse(domain_pddl, task_pddl=None):
        domain_name, requirements, types, constants, predicates, actions, axioms, sensors \
                 = parse_domain(domain_pddl)
        task = Task(domain_name, requirements, types, constants, predicates, actions, axioms, sensors=sensors)
        if task_pddl is not None:
            print "this should not work"
            self.problem = parse_task(task_pddl, task)
        return task
    
    def add_implicit_MAPL_types(self, type2supertype):
        known_types = set(t.name for t in self.types)
        unknown_types = set(type2supertype.keys()) - known_types
        for t in unknown_types:
            decl = [types.Type(tname) for tname in [t, "-", type2supertype[t]]]
            self.types.extend(decl)
    
    def add_implicit_MAPL_predicate(self, pred_decl_str):
        alist = pred_decl_str.split()
        pred_name = alist[0]
#         if pred_name not in [p.name for p in self.predicates]:
#             new_pred = predicates.Predicate.parse(alist)
#             self.predicates.add(new_pred)
#             #self.update_predicate_lists()
        if pred_name not in self.predicates:
            new_pred = predicates.Predicate.parse(alist)
            self.predicates[pred_name] = new_pred

    def add_implicit_MAPL_constants(self, typed_obj_list):
        known_constants = set(c.name for c in self.constants)
        for to in typed_obj_list:
            if to.name not in known_constants:
                self.constants.append(to)
                known_constants.add(to.name)
    
    def translate2pddl(self, subgoals=None):
        new_types = types.typed_list2dict(MAPL_BASE_ONTOLOGY.split())
        self.add_implicit_MAPL_types(new_types)
        for pred in MAPL_BASE_PREDICATES:
            self.add_implicit_MAPL_predicate(pred)
        new_objects = types.parse_typed_list(MAPL_BASE_OBJECTS.split())
        if subgoals:
            subgoal_constants = [types.TypedObject(sg, "subgoal") for sg in subgoals]
            new_objects.extend(subgoal_constants)
        self.add_implicit_MAPL_constants(new_objects)
        self.pddl_actions = [a.translate2pddl(self.predicates) for a in self.actions]
        
#     def update_predicate_lists(self):
#         self.predicate_names_boolean = dict((p.name,p) for p in self.predicates if p.is_boolean())
#         self.predicate_names_multi_valued = dict((p.name,p) for p in self.predicates if not p.is_boolean())
    
    def get_predicate_by_name(self, name):
        modal_op = None
        if name.startswith("k__") or name.startswith("kd__"):
            modal_op, name = name.split("__", 2)
        try:
            pred = self.predicates[name]
        except KeyError:
            if config.verbosity >= 3:
                print "predicate '%s' not found" % name
                if config.verbosity >= 4:
                    print "Known predicates:"
                    for pred in self.predicates:
                        print "Name: '%s'" % pred
                        print self.predicates[pred]
                        print "---"
            raise
        if modal_op:
            pred = predicates.ModalPredicate(modal_op, [], pred)
        return pred
    
    def props2svar_assignments(self, prop_list, ignore_negated_props=True):
        NOT_PREFIX = "not-"
        for prop in prop_list:
            if prop and prop[0] == "(":
                prop = prop[1:-1]
            tokens = prop.split()
            pred_name = tokens[0]
            if pred_name.startswith("__"):
                continue
            if pred_name.startswith(NOT_PREFIX):
                if ignore_negated_props:
                    continue
                # removing NOT reverses the meaning of the proposition. handle with care!
                pred_name = pred_name[len(NOT_PREFIX):]
                tokens[0] = pred_name
            prop_args = tokens[1:]
            try:
                pred = self.get_predicate_by_name(pred_name)
            except KeyError:
                if config.verbosity >= 2:
                    print "proposition '%s' could not be interpreted." % prop
                continue
            arity = pred.arity()
            if pred.is_multi_valued():
                svar, val = state.StateVariable(tokens[:arity+1]), tuple(tokens[arity+1:])
            else:
                if tokens[-1] == "true":
                    tokens = tokens[:-1] 
                svar, val = state.StateVariable(tokens), TRUE_TUP
            yield svar, val

    def get_action_by_name(self, name):
        # TODO: should cache this
        if self.name2actions is None:
            self.name2actions = {}
            for a in self.actions:
                self.name2actions[a.name.lower()] = a
        return self.name2actions.get(name.lower())

    def get_sensor_by_name(self, name):
        # TODO: should cache this
        if self.sname2sensor is None:
            self.sname2sensor = {}
            for s in self.sensors:
                self.sname2sensor[s.name.lower()] = s
        return self.sname2sensor.get(name.lower())
    
    def write_pddl_files(self, task_out_fn, dom_out_fn):
        dom_dir, dom_name = os.path.split(dom_out_fn)
        self.compile_domain_to_pddl_variants(dom_name, outdir=dom_dir)
        self.write_pddl_task(task_out_fn)
    
    def compile_domain_to_pddl_variants(self, output_fn, outdir=None):
        # setup planning domain
        if outdir is None:
            # no outdir -> compile to same directory
            outdir = os.path.dirname(mapl_domain_fn)
        output_fn = os.path.basename(output_fn)
        if output_fn.endswith(".mapl"):
            output_fn = output_fn[:-len(".mapl")]
        return_vals = []
        params = [(".pddl", False), (".cpddl", True)]
        for suffix, assertions in params:
            fn = output_fn + suffix
            fp = os.path.join(outdir, fn)
            self.write_pddl_domain(open(fp, "w"), keep_assertions=assertions)
            return_vals.append(fp)
        return return_vals     # pddl_file, cpddl_file

    def as_mapl_domain(self):
        l = [line for line in self.as_mapl_domain_gen()]
        return "\n".join(l)
        
    def as_mapl_domain_gen(self):
        yield ";; MAPL domain definition" 
        yield "(define (domain %s)" % self.domain_name
        yield self.requirements.pddl_str()
        # types
        yield types.pretty_str_typed_list(self.types)
        yield "UNFINISHED........"
        

    def write_pddl_domain(self, dom_stream, keep_assertions=False, beliefs=True):
        s = self.as_pddl_str(keep_assertions, beliefs)
        print >> dom_stream, s

    def as_pddl_str(self, keep_assertions=False, beliefs=True):
        self.translate2pddl()
        l = [line for line in self.as_pddl_str_gen(keep_assertions, beliefs)]
        return "\n".join(l)
        
    def as_pddl_str_gen(self, keep_assertions, beliefs):
        yield ";; generated by MAPL2PDDL" 
        yield "(define (domain %s)" % self.domain_name
        yield self.requirements.pddl_str()
        # types
        yield types.pretty_str_typed_list(self.types)
        # predicates
        yield "(:predicates"
        agts = [types.TypedObject("?agt0", "agent")]
        for pred in self.predicates.values():
            yield "  ;; predicate: %s" % pred.name
            declarations = []
#             if pred.name == "=":
#                 continue
            declarations.append(pred.pddl_str())
            if WRITE_IN_DOMAIN_PRED:
                idpred = pred.svar_in_domain_predicate()
                declarations.append(idpred.pddl_str())
            if beliefs:
                kpred = pred.knowledge_pred(agts)
                declarations.append(kpred.pddl_str())
                declarations.append(kpred.pddl_str(k_axiom=True))
#                 if WRITE_IN_DOMAIN_PRED:
#                     idkpred = idpred.knowledge_pred(agts)
#                     declarations.append(idkpred.pddl_str())
#                     declarations.append(idkpred.pddl_str(k_axiom=True))
            if WRITE_INITIALLY_FACTS:
                declarations.append(pred.initially_pred().pddl_str())
                if WRITE_IN_DOMAIN_PRED:
                    declarations.append(idpred.initially_pred().pddl_str())
                if beliefs:
                    declarations.append(kpred.initially_pred().pddl_str())
                    declarations.append(kpred.initially_pred().pddl_str(k_axiom=True))
#                     if WRITE_IN_DOMAIN_PRED:
#                         declarations.append(idkpred.initially_pred().pddl_str())
#                         declarations.append(idkpred.initially_pred().pddl_str(k_axiom=True))
            for decl in declarations:
                yield "  (%s)" % decl
        yield ")"
        #constants
        yield "(:constants"
        constants = set(self.constants)
        if self.planning_agent:
            agent = self.planning_agent
            aname = agent.name.lower()
            goal_str = agent.problem.goal.mapl_str()
            if hasattr(agent,"original_goal"):
                goal_str = agent.original_goal.mapl_str()
    #             print "Orig goal", goal_str
            goal_objects = agent.problem.goal_objects(goal_str)
            agent.log("Found %d goal objs: %s " % (len(goal_objects),
                                                   repr([str(o) for o in goal_objects])), vlevel=4)
            constants = constants.union(goal_objects)
            from simulation import Simulation
        for obj in constants:
            objdef = types.pddl_str(obj)
            yield "  %s" % objdef
        yield ")"
        # axioms
        for axiom in self.axioms:
            for line in axiom.pddl_gen():
                yield line
        # actions
        if self.pddl_actions is None:
            self.translate2pddl()
        for action in self.pddl_actions:
            for line in action.pddl_gen(keep_assertions):
                yield line
        # sensors
        for sensor in self.sensors:
            for line in sensor.pddl_gen(keep_assertions):
                yield line
        yield ")"

    def problem2pddl_str(self):
        return problem2pddl_str(self)

    def write_pddl_task(self, task_stream):
        problem_str = self.problem2pddl_str()
        if utils.is_string(task_stream):
            task_stream = open(task_stream, "w")
        task_stream.write(problem_str)




class Requirements(object):
    def __init__(self, requirements):
        self.requirements = requirements
        for req in requirements:
            assert req in (":strips", ":adl", ":typing", ":negation", ":equality",
            ":derived-predicates", ":mapl")
    def __str__(self):
        return ", ".join(self.requirements)
    def pddl_str(self):
        reqs = self.requirements[:]
        try:
            reqs.remove(":mapl")
        except ValueError:
            pass
        return "(:requirements %s)" % " ".join(reqs) 

done_warning = False

def parse_domain(domain_mapl):
    global done_warning
    iterator = iter(domain_mapl)
    res = []
    assert iterator.next() == "define"
    domain_line = iterator.next()
    assert domain_line[0] == "domain" and len(domain_line) == 2, \
         "Domain file is no domain file! Are you sure the first cmd line argument is the task file?"
    res.append(domain_line[1])
    
    opt_requirements = iterator.next()
    if opt_requirements[0] == ":requirements":
        res.append(Requirements(opt_requirements[1:]))
        opt_types = iterator.next()
    else:
        res.append(Requirements([":strips"]))
        opt_types = opt_requirements
    
    if opt_types[0] == ":types":
        if "object" not in opt_types:
            opt_types.append("object")
        res.append([types.Type(entry) for entry in opt_types[1:]])
        opt_constants = iterator.next()
    else:
        res.append([types.Type("object")])
        opt_constants = opt_types
    
    if opt_constants[0] == ":constants":
        res.append(types.parse_typed_list(opt_constants[1:]))
        pred = iterator.next()
    else:
        res.append([])
        pred = opt_constants
    
    preds = [predicates.Predicate.parse(entry) for entry in pred[1:]] 
    res.append(preds)
    # create 2nd-order operators, i.e. operators with state vars as parameters
    try:
        op_tmpls = config.op_templates_2nd_order
    except:
        op_tmpls = []
    autoops = [create_autoops_nested_list(pred, op_tmpl) for pred in preds for op_tmpl in op_tmpls]
    #print "created %d ops from %d preds and %d tmpl" % (len(autoops), len(preds), len(op_tmpls))
    special_ops = [parser.parse_nested_list(op) for op in SPECIAL_OPERATORS]
    iterator = itertools.chain(autoops, special_ops, iterator)
    the_axioms = []
    the_actions = []
    the_sensors = []
    for entry in iterator:
        if entry[0] == ":derived":
            axiom = axioms.Axiom.parse(entry)
            the_axioms.append(axiom)
        elif entry[0] == ":sensor":
            sensor = sensors.SensorModel.parse(entry)
            the_sensors.append(sensor)
        else:
            assert entry[0] == ":action", entry[0]
            acts = actions.Action.parse(entry, preds)
            the_actions.extend(acts)
    res.append(the_actions)
    res.append(the_axioms)
    res.append(the_sensors)
    axiom_names = set(ax.name.name for ax in the_axioms)
    if not done_warning and CAN_TALK_TO not in axiom_names:
        done_warning = True
        print "Warning: domain does not contain axiom '%s' describing preconditions for communication!" % CAN_TALK_TO
        print "Agents will not be able to negotiate multiagent plans in this domain."
        #sys.exit()
    return res

# def preprocess_parsed_facts(facts):
#     from predicates import Predicate
#     for fact in facts:
#         name = fact[0]
#         if isinstance(name, list):
#             print "\nWill crash in a sec!"
#             print name
#             print fact
#         if not name.startswith(DOMAIN_KW):
#             yield conditions.Atom(name, fact[1:])
#         else:
#             assert len(fact)==4 and fact[2] == ":", "svar domain definition does not use correct syntax '(domain (svar arg1 .. argk) : (val1 .. valn))'"
#             pred_name = fact[1][0]
#             args = fact[1][1:]
#             values = fact[3]
#             in_dom_pred = Predicate.svar_in_domain_predicate_name(pred_name)
#             for value in values:
#                 atom = conditions.Atom(in_dom_pred, args+[value], value=None)
#                 #print "new in_domain proposition: ", atom
#                 yield atom

def parse_task(task_pddl):
    iterator = iter(task_pddl)
    rlist = []
    assert iterator.next() == "define"
    problem_line = iterator.next()
    assert problem_line[0] == "problem" and len(problem_line) == 2
    rlist.append(problem_line[1])
    domain_line = iterator.next()
    assert domain_line[0] == ":domain" and len(domain_line) == 2
    rlist.append(domain_line[1])
    
    objects_opt = iterator.next()
    if objects_opt[0] == ":objects":
        rlist.append(types.parse_typed_list(objects_opt[1:]))
        init = iterator.next()
    else:
        rlist.append([])
        init = objects_opt
    
    assert init[0] == ":init"
    
    atoms = list(conditions.parse_literal(fact) for fact in init[1:])
#             yield conditions.Atom(name, fact[1:])
    
    rlist.append(atoms)
    
    goal = iterator.next()
    assert goal[0] == ":goal" and len(goal) == 2
    goal = conditions.parse_condition(goal[1])
    #print "goasl", goal
    rlist.append(goal)  
    for entry in iterator:
        assert False, entry
    return Problem(*rlist)

def create_autoops_nested_list(pred, op_tmpl):
    from string import Template
    import parser
    if not utils.is_string(op_tmpl):
        if pred.is_boolean():
            op_tmpl = op_tmpl[1]
        else:
            op_tmpl = op_tmpl[0]
    tmpl = Template(op_tmpl)
    subs = dict(svar=pred.name, args_decl=pred.args_decl_str,
                args=pred.args_str, domtype=pred.value_type_str)
    s = tmpl.safe_substitute(subs)
    nl = parser.parse_nested_list(s)
    return nl
