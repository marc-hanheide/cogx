import castinit    # this will add stuff to sys.path
import cogxv11n.core.DisplayClient as DisplayClient

from itertools import chain

import standalone
from standalone import pddl
from standalone.task import PlanningStatusEnum
from autogen import Planner


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

class PlannerDisplayClient(DisplayClient.CDisplayClient):
    def __init__(self):
        DisplayClient.CDisplayClient.__init__(self)

    # def handleEvent(self, event):
    #     if self.m_test == None: return
    #     t = self.m_test
    #     if event.sourceId == "cb.test.onoff":
    #         if event.data == "0": t.m_ckTestValue = 0
    #         else: t.m_ckTestValue = 2
    #         if t.m_ckTestValue > 0: t.appendMessage("Check box " + event.sourceId + " is ON")
    #         else: t.appendMessage("Check box " + event.sourceId + " is OFF")

    #     if event.sourceId == "button.test":
    #        t.appendMessage("Button " + event.sourceId + " PRESSED")

    # def getControlState(self, ctrlId):
    #     if self.m_test == None: return
    #     t = self.m_test
    #     if ctrlId == "cb.test.onoff":
    #         if t.m_ckTestValue != 0: return "2"
    #         else: return "0"
    #     return ""

    # def handleForm(self, id, partId, fields):
    #     if self.m_test == None: return
    #     t = self.m_test
    #     #t.log("PYTHON handleForm " + id + ":" + partId)
    #     if id == "v11n.python.setHtmlForm" and partId == "101":
    #         if fields.has_key("textfield"):
    #             t.m_textField = fields["textfield"]
    #             t.log("Got textfield: " + t.m_textField);
    #             t.appendMessage(t.m_textField);

    # def getFormData(self, id, partId, fields):
    #     if self.m_test == None: return False
    #     t = self.m_test
    #     #t.log("PYTHON getFormData " + id + ":" + partId)
    #     if id == "v11n.python.setHtmlForm" and partId == "101":
    #         fields["textfield"] = t.m_textField
    #         return True

    #     return False

    def init_html(self):
        head = "<style type=\"text/css\">%s</style>" % STYLE
        self.setHtmlHead(TASKS_ID, "head", head);
        self.setHtmlHead(STATE_ID, "head", head);

    def update_state(self, task):
        state = task.state.prob_state

        ignore = ("i_in-domain", "defined", "poss", "search_cost")

        html = "<h2>Planning state of task %d</h2>" % (task.id)

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
        
        html += make_html_table(["Variable", "Value", "p"], rows, ["class", "%s", "%s", "%.3f"])

        self.setHtml(STATE_ID, "state", html);
        
    def update_task(self, task, append=False):
        id = "%04d" % task.id
        
        html = "<h2>Planning Task %d (%s)</h2>" % (task.id, task.internal_state)

        def goal_row(g):
            if g.isInPlan:
                _class = "sat"
            elif task.status == Planner.Completion.SUCCEEDED:
                _class = "unsat"
            else:
                _class = ""
            return (_class, g.goalString, g.importance, g.isInPlan)
        
        html += make_html_table(["Goal", "Importance", "satisfied"], (goal_row(g) for g in task.slice_goals), ["class", "%s", "%d", "%s"])
        if task.dt_planning_active():
            html += "<p>Decision theoretic planner is active</p>"
        else:
            html += "<p>Continual planner is active:</p>"

        def action_row(pnode):
            args = [a.name for a in pnode.args]
            name = "(%s %s)" % (pnode.action.name, " ".join(args))
            _class = str(pnode.status).lower()
            if task.dt_task and pnode in task.dt_task.subplan_actions:
                _class += ' dt'
            return (_class, name, float(pnode.cost), float(pnode.prob), pnode.status)

        if task.cp_task.planning_status == PlanningStatusEnum.PLANNING_FAILURE:
            html += "<p>No plan found</p>"
        elif task.cp_task.get_plan():
            ordered_plan = task.cp_task.get_plan().topological_sort()
            c_total = sum(a.cost for a in ordered_plan)
            p_total = reduce(lambda x,y: x*y, (a.prob for a in ordered_plan), 1.0)
            summary = ("summary", "Total:", c_total, p_total, "")
            rows = chain((action_row(a) for a in ordered_plan), [summary])
            
            html += make_html_table(["Action", "Cost", "p", "State"], rows , ["class", "%s", "%.2f", "%.3f", "%s"])
        else:
            html += "<p>Planning (Continual)...</p>"
            
        if task.dt_planning_active():
            dt_plan = task.dt_task.dt_plan
            if dt_plan:
                html += "<h3>DT plan:</h3>"
                html += make_html_table(["Action", "Cost", "p", "State"], (action_row(a) for a in dt_plan), ["class", "%s", "%.2f", "%.3f", "%s"])
            else:
                html += "<p>Planning (DT)...</p>"

        if task.plan_history:
            html += "<h3>Plan history</h3>"
            for elem in task.plan_history:
                if elem is None:
                    html += '<p>No plan found</p>'
                else:
                    if isinstance(elem, standalone.plans.MAPLPlan):
                        ordered_plan = elem.topological_sort()
                    else:
                        ordered_plan = elem.dt_plan
                    html += make_html_table(["Action", "Cost", "p", "State"], (action_row(a) for a in ordered_plan), ["class", "%s", "%.2f", "%.3f", "%s"])
                
        
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
