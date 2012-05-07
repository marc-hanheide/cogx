import time

class PlannerLog(object):
    def __init__(self, fname):
        self.log = []
        self.fname = fname
        self.dirty = False

    def append(self, entry):
        self.log.append(entry)
        self.dirty = True

    def write(self):
        if not self.dirty:
            return
        with open(self.fname, "w") as f:
            print >>f, "(:log"
            for e in self.log:
                print >>f, e
                print >>f, ")"
            
        self.dirty = False

    def __iter__(self):
        return iter(self.log)
        

class PlannerLogEntry(object):
    def __init__(self, name):
        self.time = time.time()
        self.name = name
        self.string = None

    def __str__(self):
        if self.string is None:
            self.string = "(:%s %d %s)" % (self.name, self.time, self.to_string())
        return self.string
        

class ActionEntry(PlannerLogEntry):
    def __init__(self, node):
        PlannerLogEntry.__init__(self, "action")
        self.pnode = node
        self.status = node.status

    def to_string(self):
        argstring = " ".join(a.name for a in self.pnode.full_args)
        return "((%s %s) %s)" % (self.pnode.action.name, argstring, self.status)

class DTEntry(PlannerLogEntry):
    def __init__(self, enabled=False, cancelled=False):
        PlannerLogEntry.__init__(self, "dt")
        self.enabled = enabled
        self.cancelled = cancelled

    def to_string(self):
        if self.cancelled:
            return ":cancelled"
        return ":started" if self.enabled else ":done"

class StateEntry(PlannerLogEntry):
    def __init__(self, cast_state, goals):
        PlannerLogEntry.__init__(self, "state")
        self.state = cast_state
        self.goals = goals

    def to_string(self):
        from standalone import task, pddl
        
        problem, _, _ = self.state.to_problem(self.goals, deterministic=False)
        w = task.PDDLOutput(writer=pddl.mapl.MAPLWriter())
        _, prob_str = w.write(problem)
        return "\n".join(prob_str)

class PlanEntry(PlannerLogEntry):
    def __init__(self, plan):
        PlannerLogEntry.__init__(self, "plan")
        self.plan = plan

    def to_string(self):
        if self.plan is None:
            return ":notfound"

        lines = ("(%s %s)" % (pnode.action.name, " ".join(a.name for a in pnode.full_args))
                 for pnode in self.plan.topological_sort())
        return "\n".join(lines)

class TaskStatusEntry(PlannerLogEntry):
    def __init__(self, status):
        PlannerLogEntry.__init__(self, "status")
        self.status = status

    def to_string(self):
        return str(self.status)
    
