#! /usr/bin/env python
# -*- coding: latin-1 -*-

from collections import defaultdict
import itertools
import string
import sys


from config import log
from constants import *
import config
import graph
import operator
import os.path
import state
import utils
from utils import cached_attribute

def pddl_str2action_dict (astring):
    arguments = astring.split()
    assert len(arguments) >= 2
    return dict(operator=arguments[0], agent=arguments[1], arguments=arguments[2:])

class Action(object):
    def __init__(self, agent=None, operator=None, arguments=None, rw_description=None, planning_agent=None):
        assert not isinstance(operator, Action)
        self.operator = operator
        self.agent = agent
        self.arguments = arguments
        self.rw_description = rw_description
        self.planning_agent = planning_agent

    def copy(self):
        return Action(self.agent, self.operator, self.arguments, self.rw_description, self.planning_agent)

    @cached_attribute
    def mapl_op(self):
        try:
            domain = self.planning_agent.mapl_domain

            if self.is_sensing_action():
                return domain.get_sensor_by_name(self.operator[len(PDDL_SENSOR_PREFIX):])
            return domain.get_action_by_name(self.operator)
        except AttributeError:
            log("MAPL op access hack doesn't seem to work. Sorry.  Contact Michael B to fix it!")
            return None

    @cached_attribute
    def description(self):
        return " ".join([self.operator, self.agent] + self.arguments)

    @cached_attribute
    def action_type(self):
            if self.is_hidden():
                return VIRTUAL_ACTION
            if self.is_sensing_action():
                return SENSING_ACTION
            if self.operator == NEGOTIATE_PLAN_OP:
                return NEGOTIATION_ACTION
            if self.is_speech_act():
                return OTHER_SPEECH_ACT
            else:
                return PHYS_ACTION

    def grounded_mapl_action(self, vars_in_parens=True):
        arity = len(self.mapl_op.parameters)
        grounded_mapl = "%s: %s %s" % (self.agent, self.operator, " ".join(self.arguments[:arity]))
        if vars_in_parens and arity < len(self.arguments):
            grounded_mapl += " (%s)" % " ".join(self.arguments[arity:])
        return grounded_mapl

    def is_assertion(self):
        if not self.mapl_op or self.is_sensing_action():
            return False
        result = self.mapl_op.is_assertion()
        return result

    def is_tell_val(self):
        return self.operator.startswith("tell_val")

    def is_tell_goal(self):
        return self.operator.startswith("tell_goal")

    def is_speech_act(self):
        return self.is_tell_goal() or self.is_tell_val()

    def is_hidden(self):
        return self.operator.startswith(HIDDEN_PREFIX) and not self.operator.startswith(PDDL_SENSOR_PREFIX)

    def is_sensing_action(self):
        return self.operator.startswith(PDDL_SENSOR_PREFIX)

    def is_goal_operator(self):
        return self.operator.startswith(GOAL_PSEUDO_ACTION + PREFIX_SEP)

    def __eq__(self, obj):
        return isinstance(obj,Action) and self.description == obj.description

    def __hash__(self):
        return hash(self.description)

    def __str__(self):
        return self.description

##############################


# TODO: different ways to access node

class PO_Node(object):
    def __init__(self, action, plan=None, rw_description=None, planning_agent=None):
        if isinstance(action, Action):
            self.action = action.copy()
        else:
            self.action = Action(rw_description=rw_description, planning_agent=planning_agent, **pddl_str2action_dict(action))
        self.id = None
        self.level = 0
        self.predecessors = set()
        self.successors = set()
        self.plan = plan
    def __str__(self):
        return str(self.action)
    def is_root(self):
        return self.plan.root == self
    
    def add_successor(self, succ):
        assert succ != self, "oops: %s == %s" % (self, succ)
        if not succ.plan:
            self.plan.add(succ)
        self.successors.add(succ)
        succ.predecessors.add(self)
        succ.level = 1 + max(p.level for p in succ.predecessors)
    def closure(self, related_nodes_func):
        closed = set()
        openset = set([self])
        while openset:
            c = iter(openset).next()  # get an arbitrary element
            openset.remove(c)
            if c not in closed:
                closed.add(c)
                openset.update(related_nodes_func(c) - closed)
        return closed
    def pred_closure(self):
        return self.closure(lambda n: n.predecessors)
    def succ_closure(self):
        return self.closure(lambda n: n.successors)

class Link(object):
    def __init__(self, n1, n2, ltype=PROVIDES, labels=None):
        self.n1 = n1
        self.n2 = n2
        self.type = ltype
        self.labels = set()
        if labels:
            self.labels.update(labels)
    def __str__(self):
        return "%s -> %s [%s]" % (self.n1, self.n2, ", ".join(self.labels))
    def to_dot_label(self):
        label = ", ".join(self.labels)
        attrs = 'label="%s", %s' % (label, TYPE_ATTRIBUTES[self.type])
        return attrs

##############################

class POPlan(object):
    DUMMY_ROOT_ACTION = "dummy_root_action env"

    def __init__(self, planning_agent, id_template="a%s"):
        self.root = PO_Node(POPlan.DUMMY_ROOT_ACTION, self, planning_agent=planning_agent)
        self.all_nodes = set([self.root])
        self.root_set = set([self.root])   # helper set, contains only root
        self.id_template = id_template
        self.links = {}  # contains labels etc for links between two nodes
        self.frontier = {}   # frontier[fact] = (provider, readers)
        self.entered_time = {self.root.action : 0}  # action -> time node was added to the plan
        self.action_name2node = {}
        self.planning_agent = planning_agent
        
    def is_plan(self):
        return True

    def is_empty(self):
        return len(self.all_nodes) <= 1

    def contains(self, node=None, by_name=None):
        assert node or by_name
        if node:
            return node in self.all_nodes
        elif by_name:
            return any(n.action.description == by_name for n in self.all_nodes)
        assert False, "should not get here"

    def agents_involved(self):
        return set(n.action.agent for n in self.all_nodes)
        

    def actions_by(self, agt_name):
        return set(n.action for n in self.all_nodes if str(n.action.agent) == agt_name)

    # to be overridden in PlanningMemory
    def action_was_executed(self, action):
        return False
    def action_is_unexecutable(self, action):
        return False
    def action_in_current_plan(self, action):
        return any(action == n.action for n in self.all_nodes)

#     def rename_node(self, node, new_name):
#         # TODO: refactor
#         old_name = node.action.description
#         ent = self.entered_time[node.action]
#         # old magic that was necessary for hashing
#         del self.entered_time[node.action]
#         del self.action_name2node[old_name]
#         self.entered_time[node.action] = ent
#         self.action_name2node[new_name] = node

    def nodes(self):
        for n in self.all_nodes:
            if n == self.root:
                continue
            yield n

    def add(self, node, predecessors=None, id_template="a%s", entered=0):
        assert isinstance(node, PO_Node), "%s: %s" % (type(node) ,node)
        if node.plan is None:
            node.plan = self
        assert node.plan == self, "node belongs to another plan already!"
        node.id = id_template % string.zfill(len(self.all_nodes),2)
        self.all_nodes.add(node)
        if predecessors is None:
            predecessors = [self.root]
        if not utils.is_seq_but_not_string(predecessors):
            predecessors = [predecessors]
        for p in predecessors:
            p.add_successor(node)
        self.entered_time[node.action] = entered
        self.action_name2node[node.action.description] = node
        return node

    def remove(self, action, debug=False):
        node = self.find_node(action)
        if debug: print "removing", action
        if node is None:
            if debug: print "already gone:", action
            return
        self.all_nodes.remove(node)
        del self.action_name2node[action.description]
        for succ in node.successors:
            succ.predecessors.remove(node)
            if not succ.predecessors:
                succ.predecessors.add(self.root)
        for pred in node.predecessors:
            pred.successors.remove(node)
        self.recompute_levels_and_IDs()

    def add_link(self, n1, n2, ltype, labels, debug=False):
        if not isinstance(labels, set):
            # labels should be sets, but if they're not, make them sets...
            l  = set()
            l.add(labels)
            labels = l
        assert n1 != n2, "oops: %s links to itself because of: %s" % (n1, labels)
        assert n2 not in n1.pred_closure(), "'%s' is a predecessor of '%s', so it can't be its successor, too." % (n2, n1)
        if (n1,n2) in self.links:
            link = self.links[n1,n2]
            if link.type == ltype:
                # if link of same type already exists, extend labels
                # otherwise make new link
                link.labels.update(labels)
                return
        link = Link(n1, n2, ltype, labels)
        if debug: log("Added link: %s" % link, info=INFO_PM)
        self.links[n1,n2] = link
        n1.add_successor(n2)

    def recompute_levels_and_IDs(self):
        self.recompute_levels()
        self.recompute_ids()

##     def recompute_levels(self, nodes=None):
##         if nodes is None:
##             nodes = self.root_set
##             for n in self.all_nodes:
##                 n.level = 0
##         successors = set(s for n in nodes for s in n.successors )
##         for s in successors:
##             s.level = 1 + max(p.level for p in s.predecessors)
##         if successors:
##             self.recompute_levels(successors)

    def recompute_levels(self, nodes=None):
        log("recompute_levels()")
        if nodes is None:
            nodes = self.all_nodes - self.root_set
            for n in nodes:
                n.level = sys.maxint
            self.root.level = 0
        while nodes:
            done = set()
            for n in nodes:
                assert n.level == sys.maxint
                max_pred = max(p.level for p in n.predecessors)
                if max_pred != sys.maxint:
                    n.level = 1 + max_pred
                    done.add(n)
            if not done:
                # this should not happen usually --> cycles in the plan...
                for n in nodes:
                    print "\nnode:",str(n.action), n.level
                    print "preds:", [(str(p),p.level) for p in n.predecessors]
                # a test for cycle detection
                n = self.root
                print "\nnode:",str(n.action), n.level
                print "preds:", [(str(p),p.level) for p in n.successors]
                sys.exit()
            nodes = nodes - done

    def recompute_ids(self):
        all = self.all_nodes
        s = sorted(all, key=operator.attrgetter("level"))
        for i, node in enumerate(s):
            node.id = self.id_template % string.zfill(i,2)        
            
    def total_order(self, as_nodes=False):
        snodes = sorted(self.all_nodes, key=operator.attrgetter("level"))
        snodes = [n for n in snodes if n != self.root]
        if as_nodes:
            return snodes
        return [n.action for n in snodes]

    def to_string_list(self):
        toplan = self.total_order()
        sl = [action.description for action in toplan]
        return sl

    def get_all_actions(self, as_nodes=False):
        nodes = [node for node in self.all_nodes if node != self.root]
        if as_nodes:
            return nodes
        return [node.action for node in nodes]

    def get_level1_actions(self, as_nodes=False):
        nodes = [node for node in self.all_nodes if node.predecessors == self.root_set]
        assert all(n.level == 1 for n in nodes)
        if as_nodes:
            return nodes
        return [node.action for node in nodes]

    def find_node(self, action_name):
        if isinstance(action_name, Action):
            action_name = action_name.description
        elif isinstance(action_name, PO_Node):
            action_name = action_name.action.description
        node = self.action_name2node.get(action_name)
        return node

#     def final_state_estimation(self):
#         """return predicted end state (ie after plan execution"""
#         # TODO: why not directly use self.frontier.keys()
#         providers = [provider for provider, _ in self.frontier.values()]
#         assignments = set((svar,val) for provider in providers for (svar,val) in provider.rw_description.written_facts.items())
#         return state.State(assignments)
        
    def __str__(self):
        all = self.all_nodes - self.root_set
        sorted_items = [n for n in  sorted(all, key=operator.attrgetter("level"))]
        def gen():
            for k, group in itertools.groupby(sorted_items, key=operator.attrgetter("level")):
                group = ["(%s)" % item for item in group]
                yield "  L%d (%d actions): %s" % (k, len(group), " ".join(group))
        return "Plan = {\n%s\n}" % "\n".join(gen())

    def compute_node_label(self, node):
        return TYPE_ATTRIBUTES[PLANNED]

    def write_dot_file(self, fpath="test.dot", show_hidden_actions=False):
        g = graph.Graph()
        ranks = defaultdict(set)
        USE_RANKS = False
        for n in self.nodes():
            if not show_hidden_actions and n.action.is_hidden():
                continue
            nname = n.action.grounded_mapl_action()
            label = self.compute_node_label(n)
            g.add_node(nname, label=label)
            if USE_RANKS:
                ranks[n.level].add(nname)
            for n2 in n.successors:
                if not show_hidden_actions and n2.action.is_hidden():
                    continue
                n2name = n2.action.grounded_mapl_action()
                link = self.links[n,n2]
                if link.type == UNEXECUTABLE:
                    #must check if this has changed (a little hacky not to change the link type earlier)
                   pass 
                if self.action_is_unexecutable(n.action) or self.action_is_unexecutable(n2.action):
                    link.type = UNEXECUTABLE
                label = link.to_dot_label()
                g.add_edge(nname, n2name, label)
        gname = '"%s"' % os.path.basename(fpath)
        dot_str = g.to_dot(name=gname, ranks=ranks.values())
        f = open(fpath, "w")
        print >>f, dot_str
        f.close()

        
##############################

SET_STATE_STR = "set_state_%d env"

class PMinfo(object):
    def __init__(self):
        self.execution_time = -1
        self.unexecutable_time = -1

class PlanningMemory(POPlan):
    def __init__(self, planning_agent):
        POPlan.__init__(self, planning_agent)
        self.pm_info = defaultdict(PMinfo)
        self.task_info = utils.Struct(objects=None, goal=None, domain=None, facts=None)
        #self.virtual_actions_counter = 0
        self.current_state = state.State()
    def is_plan(self):
        return False
    def execution_time(self, action):
        assert isinstance(action, Action)
        return self.pm_info[action].execution_time
    def action_was_executed(self, action):
        assert isinstance(action, Action)
        return self.execution_time(action) > -1 or action == self.root.action
    def unexecutable_time(self, action):
        assert isinstance(action, Action)
        return self.pm_info[action].unexecutable_time
    def action_is_unexecutable(self, action):
        assert isinstance(action, Action)
        return self.unexecutable_time(action) > -1
    def action_in_current_plan(self, action):
        in_plan = POPlan.action_in_current_plan(self, action)
        exec_state = not (self.action_was_executed(action) or self.action_is_unexecutable(action))
        return in_plan and exec_state
#     def rename_node(self, node, new_name):
#         info = self.pm_info[node.action]
#         del self.pm_info[node.action]
#         POPlan.rename_node(self, node, new_name)
#         self.pm_info[node.action] = info
    def current_state_estimation(self):
        """return state as predicted by latest advance() commands"""
        return self.current_state
    def update_state(self, state, time=0):
        self.current_state.update(state)
    def set_state(self, new_state, time=0):
        self.current_state = state.State()
        self.update_state(new_state)
##     def set_state(self, state, time=0):
##         self.virtual_actions_counter += 1
##         aname = SET_STATE_STR % self.virtual_actions_counter
##         description = RWdescription(aname, {}, state)
##         make_partial_order_plan([description], time=time, pm=self)
##         anode = self.find_node(aname)
##         self.advance(anode.action)
    def compute_node_label(self, node):
        time = config.time
        action = node.action
        s = PLANNED
        if self.action_was_executed(action):
            s = EXECUTED
        elif self.action_is_unexecutable(action):
            s = UNEXECUTABLE
        return TYPE_ATTRIBUTES[s]
    def str_node(self, node):
        ent = self.entered_time[node.action]
        exe, unex = "/", "/"
        action = node.action
        if self.action_was_executed(action):
            exe = self.execution_time(action)
        if self.action_is_unexecutable(action):
            exe = self.unexecutable_time(action)
        return "%s [%s,%s,%s]" % (str(node), ent, exe, unex)        
    def add_plan(self, plan):
        log("current PM:\n%s" % self, info=INFO_PM)
        for n in self.nodes():
            if self.action_was_executed(n.action):
                continue
            if not plan.find_node(n):
                # mark nodes that were in the old plan, but are not in the new one as unexecutable
                self.mark_as_unexecutable(n)
                continue
            elif self.action_is_unexecutable(n.action):
                #print "re-adding", n
                self.mark_as_in_plan(n)
            self.clean_action_effects(n)
        # add actions in an executable order --> build TO of plan first
        sorted_nodes = sorted(plan.nodes(), key=operator.attrgetter("level"))
        action_descriptions = [n.action.rw_description for n in sorted_nodes]
        make_partial_order_plan(action_descriptions, self.planning_agent, time=config.time, pm=self)
        log("updated PM:\n%s" % self, info=INFO_PM)
    def mark_as_in_plan(self, node):
        # TODO: keep track of when it was re-added
        self.pm_info[node.action].unexecutable_time = -1
    def mark_as_unexecutable(self, node, predecessors_too=True):
        log("marking %s as unexecutable" % (node), info=INFO_PM)
        action = node.action
        if self.action_is_unexecutable(action):
            return
        self.pm_info[node.action].unexecutable_time = config.time
        self.clean_action_effects(node)
        if predecessors_too:
            self.mark_predecessors_as_unexecutable(node)
    def clean_action_effects(self, node):
        action = node.action
        _, read_vars, readonly_vars, written_vars = action.rw_description.as_tuple()
        for a in written_vars:
            provider, other_readers = self.frontier.setdefault(a, (self.root,[]))
            if provider == node:
                del self.frontier[a]
##             else:
##                 print "cleaning effects of", action
##                 print "types", type(node), type(provider)
##                 print "'%s' added by action, but 'provided' by %s" % (a, provider) 
        for ro in readonly_vars:
            provider, other_readers = self.frontier.setdefault(ro, (self.root,[]))
            try:
                other_readers.remove(node)
            except ValueError:
                pass
    def restrict_to_nodes(self, nodes, return_plan=False):
        poplan = POPlan(self.planning_agent) if return_plan else PlanningMemory(self.planning_agent)
        node2action = dict((n,n.action) for n in nodes)
        action2nnode = {}
        for n in nodes:
            action = n.action
            log("using action %s" % (action), info=INFO_PM)
            node = PO_Node(action, poplan, rw_description=action.rw_description)
            action2nnode[action] = node
            poplan.add(node, entered=self.entered_time[action])
            if not return_plan:
                pm_info = poplan.pm_info[action]
                if self.action_was_executed(action):
                    pm_info.execution_time = self.execution_time(n)
                if self.action_is_unexecutable(n.action):
                    pm_info.unexecutable_time = self.unexecutable_time(n)
        for n in nodes:
            for succ in n.successors:
                if succ not in nodes:
                    continue
                action1 = n.action
                action2 = succ.action
                nn1 = action2nnode[action1]
                nn2 = action2nnode[action2]
                link = self.links[n,succ]
                poplan.add_link(nn1, nn2, link.type, link.labels)
                log("added link between %s and %s" % (nn1, nn2), info=INFO_PM)
        poplan.recompute_levels_and_IDs()
        return poplan
    def current_plan(self):
        log("PM before extracting of current_plan():\n%s" % self, info=INFO_PM)
        nodes = set(n for n in self.all_nodes if self.action_in_current_plan(n.action))
        plan = self.restrict_to_nodes(nodes, return_plan=True)
        log("current_plan(): \n%s" % plan, info=INFO_PM)
        return plan
    def advance(self, action, time=None):
        if time is None:
            time = config.time
        anode = self.find_node(action)
        if anode is None:
            raise Exception("Add action '%s' to plan first! Plan is:\n%s" % (action, self))
        if any(self.action_in_current_plan(pred.action) for pred in anode.predecessors):
            log("Oops: executable predecessors: %s" % 
                [str(a) for a in anode.predecessors if self.action_in_current_plan(a.action)])
            print "trying to advance action", action
            raise Exception("Oops: executable predecessors: %s" % 
                            [str(a) for a in anode.predecessors if self.action_in_current_plan(a.action)])
        action = anode.action
        self.current_state.update(action.rw_description.written_facts)
        self.pm_info[action].execution_time = time
        anode = self.find_node(action)
        self.mark_predecessors_as_unexecutable(anode)
        self.recompute_levels_and_IDs()
    def execution_history(self):
        nodes = [n for n in self.nodes() if self.action_was_executed(n.action)]
        actions = [n.action.description for n in nodes]
        nodes.sort(key=lambda n:self.execution_time(n.action))
        return [str(n) for n in nodes]
    def mark_predecessors_as_unexecutable(self, anode):
        # mark all predecessors as unexecutable (if not marked somehow already)
        for n in anode.pred_closure():
            if self.action_in_current_plan(n.action):
                self.mark_as_unexecutable(n, predecessors_too=False)


############################################
# general helper functions
############################################

class RWdescription(object):
    def __init__(self, name, readonly_facts, written_facts, read_facts={}):
        self.name = name
        # TODO: read_facts vs readonly_facts. can we live with only one of them?
        self.readonly_facts = readonly_facts
        self.written_facts = written_facts
        self.read_facts = read_facts
    def as_tuple(self):
        return self.name, self.read_facts, self.readonly_facts, self.written_facts

NOT_PREFIX = "not-"

def analyse_pddl_action(name, preconds, adds, dels):
    """
    Determine which state variables are read and written by an operator
    by analyzing its STRIPS-like precondtions, add and delete effects.
    """
    log("\nAnalysing action %s" % name, info=INFO_PARTIAL_ORDER)
    def remove_neg_effects(sv):
        if sv.name.startswith(NOT_PREFIX):
            sv.name = sv.name[len(NOT_PREFIX):]
            #sv.args = sv.args[:-1]
        return sv
    log("pre: %s" % [str(var) for var in preconds], info=INFO_PARTIAL_ORDER)
    log("add: %s" % [str(var) for var in adds], info=INFO_PARTIAL_ORDER)
    log("del: %s" % [str(var) for var in dels], info=INFO_PARTIAL_ORDER)
    read = set(sv for sv in preconds)
    written_vars = set(adds.keys() + dels.keys())
    written = set(remove_neg_effects(sv) for sv in written_vars)
    log("written: %s" % [str(var) for var in written], info=INFO_PARTIAL_ORDER)
#     written = set(remove_neg_effects(sv) for sv in adds or dels)
    readonly = read - written
    readonly_facts = dict((sv, preconds[sv]) for sv in readonly)
    written_facts = dict((sv, adds[sv]) for sv in written)
    read_facts = dict((sv, preconds[sv]) for sv in read)
    descs = dict(read_facts=read_facts, readonly_facts=readonly_facts, written_facts=written_facts)
    for desc_name in descs:
        # only for logging
        desc = descs[desc_name]
        desc_strs = ["%s : %s" % (str(sv), str(desc.get(sv))) for sv in desc]
        log("%s: %s" % (desc_name, desc_strs), info=INFO_PARTIAL_ORDER)
    return RWdescription(name, readonly_facts, written_facts, read_facts)

def analyse_mapl_action(name, preconds, effects):
    read = set(sv for sv in preconds)
    written = set(sv for sv in effects)
    readonly = read - written
    readonly_facts = dict((sv, preconds[sv]) for sv in readonly)
    written_facts = dict((sv, effects[sv]) for sv in written)
    read_facts = dict((sv, preconds[sv]) for sv in read)
    return RWdescription(name, readonly_facts, written_facts, read_facts)

def rw_descriptions_from_ff(plan, action_descriptions):
    assert plan is None or action_descriptions is not None
    if action_descriptions is not None:
        return [analyse_pddl_action(*description) for description in action_descriptions]
    return None

def rw_descriptions_from_mapl(plan, domain):
    rw_descriptions = []
    #print planning_agent.domain_data.pddl_domain_fn
    #print planning_agent.mapl_domain.name2actions.keys()
    for action in plan:
        #print "\nnext action:", action
        elmts = action.split()
        op_name, args = elmts[0], elmts[1:]

        if op_name.startswith(PDDL_SENSOR_PREFIX):
            sname = op_name.replace(PDDL_SENSOR_PREFIX,"")
            sensor = domain.sname2sensor[sname]
            preconditions, effects = sensor.grounded_read_write_data(args)
        else:
            operator = domain.get_action_by_name(op_name)
            preconditions, effects = operator.grounded_read_write_data(args)
#         print ["%s : %s" % (sv,preconditions[sv]) for sv in preconditions]
#         print ["%s : %s" % (sv,effects[sv]) for sv in effects]
        rw_descriptions.append(analyse_mapl_action(action, preconditions, effects))
    return rw_descriptions;
    
def extract_plan(plan, action_descriptions, planning_agent, make_PO=True):    
    if config.action_semantics_from_ff:
        rw_descriptions = rw_descriptions_from_ff(plan, action_descriptions)
    else:
        rw_descriptions = rw_descriptions_from_mapl(plan, planning_agent.mapl_domain)

    if make_PO and action_descriptions is not None:
        return make_partial_order_plan(rw_descriptions, planning_agent)
    else:
        return make_totally_ordered_POPlan(plan, planning_agent)

asmt2str = state.str_assignment

def make_partial_order_plan(action_descriptions, planning_agent, time=0, pm=None):
    """Find a partial-order plan to which action_descriptions is a total order.
       Each action_description is a tuple (name, preconds, adds, dels).
       No treatment of conditional effects yet!"""
    if action_descriptions is None:
        return None
    if pm is None:
        poplan = POPlan(planning_agent)
    else:   # add to PM
        poplan = pm
    debug = pm and pm.is_empty()
    frontier = poplan.frontier
    root_as_default_provider = False
    default_provider = poplan.root
    wstate = planning_agent.world_state
    for sv in wstate:
        val = wstate[sv][0]
        frontier[sv] = (poplan.root, [])
    toplan = [rw_desc.name for rw_desc in action_descriptions]
    log("\nPostprocessing the following TO plan:\n%s\n" % toplan, info=INFO_PARTIAL_ORDER)
    for description in action_descriptions:
        assert isinstance(description, RWdescription), "strange: %s" % str(description)
        name, read_vars, readonly_vars, written_vars = description.as_tuple()
        log("Next action to be integrated: %s" % name, info=INFO_PARTIAL_ORDER)
        # check if action is already in the plan or PM
        node = poplan.find_node(name)
        if node and not poplan.action_in_current_plan(node.action):
            # action was already executed or is marked as unexecutable --> re-add as new node
            log("Deprecated code.  Please fix usage in plans.py.", vlevel=2, info=INFO_PARTIAL_ORDER)
#             print "Node", node
#             print "Plan", poplan.current_plan()
#             print "PM", poplan
#             print "Deprecated code.  Please fix usage in plans.py."
#             poplan.rename_node(node, node.action.description+"*")
#             node = None
        if node is None:
            node = PO_Node(name, poplan, rw_description=description, planning_agent=planning_agent)
            poplan.add(node, entered=time)
        for ro in read_vars:
            val = read_vars[ro]
            provider, other_readers = frontier.setdefault(ro, (default_provider,[]))
            poplan.add_link(provider, node, PROVIDES, asmt2str(ro,val), debug)
            log("'%s' reads fact '%s' provided by '%s'." % (node, asmt2str(ro,val), provider), info=INFO_PARTIAL_ORDER)
            other_readers.append(node)
        for w in written_vars:
            val = written_vars[w]
            provider, other_readers = frontier.setdefault(w, (default_provider,[]))
            if provider == node:
                # TODO: this is problematic! have a look
                log("strange things happen here. debug me!", vlevel=1, info=INFO_PARTIAL_ORDER)
            frontier[w] = (node, [])
            log("'%s' writes '%s' previously written by '%s'" % (node, asmt2str(w,val), provider), info=INFO_PARTIAL_ORDER)
            if provider != poplan.root:
                # do we need this at all?
                poplan.add_link(provider, node, PROVIDES, asmt2str(w,val), debug)
            for r in other_readers:
                if r == node:
                    continue
                old_val = r.action.rw_description.readonly_facts.get(w)
                # if in a PLAN the action 'name' threatens action r, 'name' must come after r
                # in a PLANNING_MEMORY this is only true if r is part of the current plan (not already executed or unexecutable)
                if (poplan.is_plan() or poplan.action_in_current_plan(r.action)):
                    log("'%s' must come before '%s' because the latter threatens the former" % (r, node), info=INFO_PARTIAL_ORDER)
                    poplan.add_link(r, node, PREVENTS_THREAT, asmt2str(w,old_val), debug)
                elif poplan.action_is_unexecutable(r.action):
                    log("'%s' would have been threatened by '%s' (but is already removed)" % (r, node), info=INFO_PARTIAL_ORDER)
                    #poplan.add_link(node, r, THREATENED_BY, asmt2str(w,val), debug)
                elif poplan.action_was_executed(r.action):
                    # don't care about the old stuff any more here...
                    pass
        if not node.predecessors:
            print "should not be here at all"
            poplan.add_link(default_provider, node, PROVIDES, "", debug)
        if not root_as_default_provider:
            default_provider = node
    poplan.recompute_levels_and_IDs()
    log("Partially ordered plan:\n%s" % poplan, info=INFO_PARTIAL_ORDER)
    return poplan

def make_partial_order_plan_from_action_seq(seq):
    assert all(a.rw_description for a in seq)
    poplan = make_partial_order_plan((a.rw_description for a in seq))
    return poplan

def make_totally_ordered_POPlan(seq, planning_agent):
    if seq is None:
        return None
    p = POPlan(planning_agent)
    last = p.root
    for action in seq:
        node = p.add(action, predecessors=[last])
        last = node
    return p

def get_relevant_svars(plan):
    relevant_now = set()
    possibly_relevant = set()
    
    for node in plan.nodes():
        read = set(node.action.rw_description.read_facts.keys())
        possibly_relevant |= read
        prev_written = set()
        #print "  action:", node.action
        #print "  read:", [str(var) for var in read]

        for pred in node.pred_closure():
            if (pred.action.description.startswith(PDDL_SENSOR_PREFIX)
                or pred == plan.root or pred == node):
                continue
            #print "    action:", pred.action
            written = set(pred.action.rw_description.written_facts.keys())
            prev_written |= written
            #print "    written:", ["%s=%s" % (str(var), str(val)) for var, val in pred.action.rw_description.written_facts.iteritems()]
            
        relevant_now |= read - prev_written
    
    #print " possibly", [ str(var) for var in possibly_relevant]
    #print " certainly", [ str(var) for var in relevant_now]
    return relevant_now

