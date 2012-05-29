import castinit    # this will add stuff to sys.path
import cogxv11n.core.DisplayClient as DisplayClient

from itertools import chain

import standalone
from standalone import pddl
from standalone.task import PlanningStatusEnum
from autogen import Planner
from cast_task import TaskStateInfoEnum, TaskStateEnum

TASKS_ID = "planner.tasks"
STATE_ID = "planner.state"

STYLE = """
tr.sat {background-color: #88FF88}
tr.unsat {background-color: #FF6666}

tr.det td:nth-child(3) {visibility:hidden;}
tr.prob td:nth-child(1) {visibility:hidden;}
tr.sep {visibility:hidden; height:5px}
tr.sep td {display:none;}

tr.executed {background-color: #CCCCCC}
tr.failed {background-color: #FF6666}
tr.in_progress {background-color: #88FF88}
tr.summary {font-weight: bold; font-size: 110%}
tr.dt {background-color: #8888ff}
tr.dt.in_progress {background-color: #88ffff}
tr.dt.executed {background-color: #6666dd}
"""

STATE_HTML = """
<h2>Planning state of task %(task_id)d</h2>
<p>
%(table)s
</p>
"""

TASK_HTML = """
<h2>Planning Task %(task_id)d (%(internal_state)s)</h2>
%(desc)s

%(goals)s
Deadline: %(deadline)s

%(planner_type)s

%(current_plan)s

<h3>Plan history:</h3>
%(history)s

"""

class PlannerDisplayClient(DisplayClient.CDisplayClient):
    def __init__(self):
        DisplayClient.CDisplayClient.__init__(self)

    def init_html(self):
        head = "<style type=\"text/css\">%s</style>" % STYLE
        self.setHtmlHead(TASKS_ID, "head", head);
        self.setHtmlHead(STATE_ID, "head", head);

    def update_state(self, task):
        state = task.state.prob_state

        ignore = ("i_in-domain", "defined", "poss", "search_cost")

        def var_row(svar, dist):
            if dist.value is not None:
                yield ("det", str(svar), str(dist.value.name), 1.0)
                return
            
            first = True
            for val, p in sorted(dist.iteritems(), key=lambda (v,_p): -_p):
                if val == pddl.UNKNOWN:
                    continue
                
                if first:
                    yield ("prob-first", str(svar), str(val.name), p)
                    first = False
                else:
                    yield ("prob", str(svar), str(val.name), p)

        rows = []
        prev_function = None
        for svar, dist in sorted(state.iterdists(), key=lambda (v,p): str(v)):
            if svar.function.name in ignore or (svar.modality and svar.modality.name in ignore):
                continue
            if (svar.function, svar.modality) != prev_function:
                rows.append(("sep", "", "", 0.0))
                prev_function = (svar.function, svar.modality)
            rows += list(var_row(svar, dist))

        task_id = task.id
        table = make_html_table(["Variable", "Value", "p"], rows, ["class", "%s", "%s", "%.3f"])
        html = STATE_HTML % locals()

        self.setHtml(STATE_ID, "state", html);


    def get_task_state_desc(self, state_info):
        if state_info == TaskStateInfoEnum.PLANNING_CP:
            return "Continual planner is working"
        if state_info == TaskStateInfoEnum.PLANNING_DT:
            return "Decision theoretic planner is working"
        if state_info == TaskStateInfoEnum.WAITING_FOR_ACTION:
            return "Waiting for action executon to finish"
        if state_info == TaskStateInfoEnum.WAITING_FOR_CONSISTENT_STATE:
            return "Waiting for world state to become consistent"
        if state_info == TaskStateInfoEnum.WAITING_FOR_EFFECT:
            return "Waiting for expected action effects to appear"
        if state_info == TaskStateInfoEnum.WAITING_FOR_OBSERVATION:
            return "Waiting for expected observations to arrive"
        if state_info == TaskStateInfoEnum.EXECUTION_FAILURE:
            return "Action execution failed"
        if state_info == TaskStateInfoEnum.PLANNING_FAILURE:
            return "Planning failed"
        if state_info == TaskStateInfoEnum.NO_CONSISTENT_STATE:
            return "World state is inconsistent."
        if state_info == TaskStateInfoEnum.INVALID_GOAL:
            return "Cannot parse goal"
        if state_info == TaskStateInfoEnum.EXPLANATIONS_PENDING:
            return "Searching for explanations of failure"
        if state_info == TaskStateInfoEnum.EXPLANATIONS_FOUND:
            return "Found possible explanations for failure"
        if state_info == TaskStateInfoEnum.EXPLANATIONS_FOUND:
            return "Found no explanations for failure"
        return ""
        
        
    def update_task(self, task, info_status=None, append=False):
        id = "%04d" % task.id

        task_id = task.id
        internal_state = task.internal_state
        desc = self.get_task_state_desc(info_status)
        desc = "<h3>%s</h3>" % desc if desc else ""

        def goal_row(g):
            if g.isInPlan:
                _class = "sat"
            elif task.status == Planner.Completion.SUCCEEDED:
                _class = "unsat"
            else:
                _class = ""
            return (_class, g.goalString, g.importance, g.isInPlan)

        goals = make_html_table(["Goal", "Importance", "satisfied"], (goal_row(g) for g in task.slice_goals), ["class", "%s", "%d", "%s"])
        deadline = "%d" % task.cp_task.deadline if task.cp_task.deadline is not None else "None"
        
        planner_type = ""
        if task.internal_state not in (TaskStateEnum.INITIALISED, TaskStateEnum.FAILED, TaskStateEnum.COMPLETED):
            if task.dt_planning_active():
                planner_type = "<p>Decision theoretic planner is active</p>"
            else:
                planner_type = "<p>Continual planner is active:</p>"

        def action_row(pnode):
            args = [a.name for a in pnode.args]
            name = "(%s %s)" % (pnode.action.name, " ".join(args))
            _class = str(pnode.status).lower()
            if task.dt_task and pnode in task.dt_task.subplan_actions:
                _class += ' dt'
            return (_class, name, float(pnode.cost), float(pnode.prob), pnode.status)

        if task.cp_task.planning_status == PlanningStatusEnum.PLANNING_FAILURE:
            current_plan = "<p>No plan found</p>"
        elif task.cp_task.get_plan():
            ordered_plan = task.cp_task.get_plan().topological_sort()
            c_total = sum(a.cost for a in ordered_plan)
            p_total = reduce(lambda x,y: x*y, (a.prob for a in ordered_plan), 1.0)
            summary = ("summary", "Total:", c_total, p_total, "")
            rows = chain((action_row(a) for a in ordered_plan), [summary])
            
            current_plan = make_html_table(["Action", "Cost", "p", "State"], rows , ["class", "%s", "%.2f", "%.3f", "%s"])
        else:
            current_plan = "<p>Planning (Continual)...</p>"
            
        if task.dt_planning_active():
            dt_plan = task.dt_task.dt_plan
            if dt_plan:
                current_plan = "<h3>DT plan:</h3>"
                current_plan += make_html_table(["Action", "Cost", "p", "State"], (action_row(a) for a in dt_plan), ["class", "%s", "%.2f", "%.3f", "%s"])
            else:
                current_plan = "<p>Planning (DT)...</p>"

        history = ""
        if task.plan_history:
            for elem in task.plan_history:
                if elem is None:
                    history += '<p>No plan found</p>'
                else:
                    if isinstance(elem, standalone.plans.MAPLPlan):
                        ordered_plan = elem.topological_sort()
                    else:
                        ordered_plan = elem.dt_plan
                    history += make_html_table(["Action", "Cost", "p", "State"], (action_row(a) for a in ordered_plan), ["class", "%s", "%.2f", "%.3f", "%s"])
                

        html = TASK_HTML % locals()
                    
        # A multi-part HTML document.
        # Parts will be added every time the form (setHtmlForm below) is submitted (see handleForm).
        if append:
            self.setHtml(TASKS_ID, id, html);
        else:
            self.setHtml(TASKS_ID, "0000", html);

        # Test of gui elements
        # Messages will be added to the document when events happen (see handleEvent).
        #self.m_display.addCheckBox("v11n.python.setHtml", "cb.test.onoff", "Test On Off");
        #self.m_display.addButton("v11n.python.setHtml", "button.test", "Test Button");

    def remove_task(self, task):
        self.update_task(task, append=True)
        #id = "%04d" % task.id
        #self.setHtml(TASKS_ID, id, "");

    def makeHtmlForm(self):
        # A simple form.
        # Events will be handled in MyDisplayClient.handleForm().
        # Form data will be retreived in MyDisplayClient.getFormData().
        self.m_display.setHtmlForm("v11n.python.setHtmlForm", "101",
              "Edit me: <input type='text' name='textfield' value='Empty' />");

    # Called from handleForm() (after a form is submitted)
    # Appends a text message (a html chunk) to an HTML object.
    # Chunks are sorted by their string ID (but the order depends on C++ std::map implementation).
    def appendMessage(self, message):
        self._msgid += 1
        self.m_display.setHtml("v11n.python.setHtml", "%04d" % self._msgid, "<br>" + message);
    
def make_html_table(names, entries, formats = None):
    head = "<table>" + "".join("<th>%s</th>" % n for n in names)
    if not formats:
        formats = ["%s"] * len(names)

    if formats[0] == "class":
        row_template = "<tr class=\"%s\">" + "".join("<td>%s</td>" % f for f in formats[1:]) + "</tr>"
    else:
        row_template = "<tr>" + "".join("<td>%s</td>" % f for f in formats) + "</tr>"

    lines = [head]
    for e in entries:
        lines.append(row_template % e)
    lines.append("</table>")
    return "\n".join(lines)
