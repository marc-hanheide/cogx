#! /usr/bin/env python
# -*- coding: latin-1 -*-

import plans

def make_state(state, factdict):
    nstate = set()
    for fact in state:
        if fact[0] == '(':
            fact = fact[1:-1]
        cached_fact = factdict.setdefault(fact, fact)
        nstate.add(cached_fact)
    return nstate

class TrieNode(object):
    def __init__(self, last_action):
        self.last_action = last_action
        self.state = set()
        self.branches = {}
        self.parent = None
    def set_node_state(self, state, factdict):
        nstate = make_state(state, factdict)
        self.state = nstate

class PlanningMemory(object):
    def __init__(self):
        self.root = TrieNode(None)
        self.now = self.root
        self.history = []
        self.change_log = []
        self.all_facts = {}
            
    def __repr__(self):
        return "\n".join(pretty_print(self.tree_repr()))

    def add_plan(self, seq, log=None):
        if isinstance(seq, plans.POPlan):
            seq = seq.total_order()
        empty_states = [set() for a in seq]
        self.add_plan_and_states(seq, empty_states, log)

    def add_plan_and_states(self, seq, states, log=None):
        #print "add_plan_and_states"
        if isinstance(seq, plans.POPlan):
            seq = seq.total_order()
        if len(states) == len(seq)+1:
            self.set_current_state(states[0])
            states = states[1:]
        current_node = self.now
        for state, elmt in zip(states, seq):
            action = elmt #Action(elmt)
            #nah: removed here
            #self.set_state(state, node=current_node)
            try:
                next_node = current_node.branches[action]
            except KeyError:
                next_node = TrieNode(action)
                current_node.branches[action] = next_node
                next_node.parent = current_node
            current_node = next_node
            #nah: inserted here
            self.set_state(state, node=current_node)
        if not self.history or current_node != self.history[-1]:
            self.history.append(current_node)
            self.change_log.append(log)

    def contains(self, seq):
        seq = list(seq) + [None]  # mark end
        current_node = self.root
        for elmt in seq:
            action = elmt #Action(elmt)
            try:
                current_node = current_node.branches[action]
            except KeyError:
                return False
        return True

    def longest_prefix(self, seq):
        seq = list(seq) + [None]  # mark end
        current_node = self.root
        for elmt in seq:
            action = elmt #Action(elmt)
            try:
                current_node = current_node.branches[action]
            except KeyError:
                return self.seq_backwards(current_node)
        return seq[:-1]

    def tree_repr(self, node=None):
        if node is None:
            node = self.root
        subtree = [self.tree_repr(node.branches[next_elmt]) for next_elmt in node.branches]
        if len(subtree) == 1:
            subtree = subtree[0]
        l = [node.last_action]
        if node == self.now:
           l.append("*NOW*")
        return l + subtree

    def seq_backwards(self, node, start_node=None, return_nodes=False):
        l = []
        while node not in (self.root, start_node, None):
            if return_nodes:
                l.append(node)
            else:
                l.append(node.last_action)
            node = node.parent
            if node == start_node:
                break
        l.reverse()
        return l

    def all_entries(self, unique=False):
        nodes = self.history
        if unique:
            nodes = set(nodes)
        return [self.seq_backwards(node.parent) for node in nodes]           

    def get_next_step(self):
        nodes = self.current_plan_seq(return_nodes=True)
        if not nodes:
            return None
        action = nodes[0].last_action
        return action

    def current_plan_seq(self, return_nodes=False):
        if not self.history:
            return []
        end = self.history[-1]
        if end.last_action is None:
            end = end.parent
        return self.seq_backwards(end, self.now, return_nodes=return_nodes)

    def current_plan(self, return_nodes=False):
        plan = self.current_plan_seq(return_nodes)
        return plans.make_partial_order_plan_from_action_seq(plan)
        #return plans.make_totally_ordered_POPlan(plan)

    def get_history(self):
        return self.seq_backwards(self.now)
    
    def advance(self, step=None):
        if step is None:
            step = self.get_next_step()
        if step is None:
            raise Exception("No next step in plan memory!")
        oldnow = self.now
        try:
            self.now = self.now.branches[step]
        except KeyError:
            # TODO: hack to clean PM when unknown action is executed
            self.__init__()
        #return self.now.state   # return new state
        return self.now.state - oldnow.state  # return difference between states

    def set_state(self, state, node=None):
        if node is None:
            node = self.now
        node.set_node_state(state, self.all_facts)

    def set_current_state(self, state):
        #print "set_current_state"
        nstate = make_state(state, self.all_facts)
        if nstate != self.now.state:
            self.add_plan_and_states(["STATE_UPDATE env"], [nstate])
            self.advance()
            #print "PM now looks like this\n", self
            #sys.exit()
        else:
            #print "no update of current state necessary"
            pass

    
def list_or_tuple(elmt):
    return isinstance(elmt, (list, tuple))

def string_tuple_or_string(elmt):
    return isinstance(elmt, (list, tuple, basestring))

def pretty_print(seq_of_seqs, indent=0, step_inside=list_or_tuple):
    l = []
    tmp = []
    for elmt in seq_of_seqs:
        if elmt is None:
            elmt = "|"
        if step_inside(elmt):
            if tmp:
                l.append(indent*' '+" ".join(tmp))
                indent = len(l[-1])+1
                tmp = []
            l.extend(pretty_print(elmt, indent, step_inside))
        else:
            elmt = str(elmt)
            if ' ' in elmt:
                elmt = "'%s'" % elmt
            tmp.append(elmt)
    if tmp:
        l.append(indent*' '+" ".join(tmp))
    return l

def trie_test():
    t = PlanningMemory()
    t.add_plan("Michael")
    t.add_plan("Anne")
    t.add_plan("Michelangelo")  # shares prefix "Mich" with some of the others
    t.add_plan("Michelangelo")  # should not appear twice
    print "trie:\n", t
    print "all entries", t.all_entries()
    t.add_plan("Michaela")  # should still show "Michael" as distinctive entry
    print "trie:\n", t
    print "all entries", t.all_entries()
    assert not t.contains("Mama")
    print "contains 'Mama'?", t.contains("Mama")
    assert t.contains("Michael")
    print "contains 'Michael'?", t.contains("Michael")
    s = "Anjelica"
    print "longest prefix for '%s':" % s, t.longest_prefix(s)
    s = "Michael"
    print "longest prefix for '%s':" % s, t.longest_prefix(s)
    s = "Mick Jagger"
    print "longest prefix for '%s':" % s, t.longest_prefix(s)

def main():
    pm = PlanningMemory()
    pm.set_state("pred1 aaa bbb ccc.pred2 bbb ccc ddd.pred3 ccc ddd eee fff".split('.'))
    first_state = pm.now.state
    plan = ["move A B", "pickup cube B", "move B A", "drop cube A"]
    print "First plan is", plan
    pm.add_plan(plan)
    print "PM now looks like this\n", pm
    #print "'Now' node is: ", pm.now.last_action
    next_step = pm.get_next_step()
    print "The next step in PM is:", next_step
    print "\nWe now notify the PM that this step has been executed:"
    pm.advance()
    pm.set_state("pred1 aaa bbb ccc.pred2 bbb ccc ddd.pred4 ccc ddd eee fff".split('.'))
    #print "'Now' node is: ", pm.now.last_action
    print "PM now looks like this\n", pm
    next_step = pm.get_next_step()
    print "\nThe next step in PM is:", next_step
    current_plan = pm.current_plan()
    print "The remaining plan is:", current_plan 
    print "\nLet's see whether adding it again changes the tree structure"
    pm.add_plan(current_plan)
    print "PM now looks like this\n", pm
    print "\nLet's change our minds and switch to another plan:"
    #pm.advance()
    pm.add_plan(["pickup cube B", "move B C", "drop cube C"], log="C seems a more promising location")
    print "PM now looks like this\n", pm
    print "But, no, let's go back to the first plan"
    pm.add_plan(["pickup cube B", "move B A", "drop cube A"], log="human explicitly asked for A as goal location")
    print "In the tree this revision is not visible:\n", pm
    print "However, the PM can effiently store these things too. The plan history looks like this:"
    for plan in pm.all_entries():
        print plan
    print "\nWhat happened until now?", pm.get_history()
    print "What will happen in the future?", pm.current_plan()
    print "\nAll entries in the PM:", pm.all_entries()
    print "Now again, but duplicates removeded:", pm.all_entries(unique=True)
    print "\nNow let's advance through the rest of the plan."
    for step in pm.current_plan():
        pm.advance()
    print "PM now looks like this\n", pm
    current_plan = pm.current_plan()
    print "The remaining plan is:", current_plan 
    #pm.advance()    # this will raise an exception!
    addition = ["some extension", "of the current plan"]
    print "The plan's empty, but we can extend it with:", addition, "and advance one step."
    pm.add_plan(addition, log="new goal lead to new plan")
    pm.advance()    # this will raise an exception!
    pm.set_state("pred1 aaa bbb ccc.pred2 bbb ccc ddd.pred3 ccc ddd eee fff".split('.'))
    print "PM now looks like this\n", pm
    print "\nHey, why did the system change it's plans all the time? Let's look at the reasons:"
    history = pm.all_entries()
    assert len(history) == len(pm.change_log)
    for i, plan in enumerate(history):
        reason = pm.change_log[i]
        if reason:
            print "was changed for the following reason:", reason
        print "plan '%s'" % str(plan)
    # states
    print "first state:  ", first_state
    print "current state:", pm.now.state
    print "IDs:  ", [id(f) for f in first_state]
    print "IDs:  ", [id(f) for f in pm.now.state]
if __name__ == "__main__":
    #trie_test()
    main()
