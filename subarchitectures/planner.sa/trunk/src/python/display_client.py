import castinit    # this will add stuff to sys.path
import cast.core
import cogxv11n.core.DisplayClient as DisplayClient

from standalone.task import PlanningStatusEnum
from autogen import Planner


TASKS_ID = "planner.tasks"

STYLE = """
tr.sat {background-color: #88FF88}
tr.unsat {background-color: #FF6666}

tr.executed {background-color: #CCCCCC}
tr.failed {background-color: #FF6666}
tr.executing {background-color: #88FF88}
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

        
    def update_task(self, task):
        id = "%04d" % task.id
        
        html = "<h2>Planning Task %d (%s)</h2>" % (task.id, task.status)

        def goal_row(g):
            if task.status == Planner.Completion.SUCCEEDED:
                if g.isInPlan:
                    _class = "sat"
                else:
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
            return (str(pnode.status).lower(), name, float(pnode.cost), pnode.status)
            
        if task.cp_task.planning_status == PlanningStatusEnum.PLAN_AVAILABLE:
            ordered_plan = task.cp_task.get_plan().topological_sort()
            html += make_html_table(["Action", "Cost", "State"], (action_row(a) for a in ordered_plan), ["class", "%s", "%.2f", "%s"])
        elif task.cp_task.planning_status == PlanningStatusEnum.RUNNING:
            html += "<p>Planning...</p>"
        elif task.dt_planning_active():
            dt_plan = task.dt_task.dt_plan
            html += "DT plan:"
            html += make_html_table(["Action", "Cost", "State"], (action_row(a) for a in dt_plan), ["class", "%s", "%.2f", "%s"])
        else:
            html += "<p>No plan found</p>"

        
        # A multi-part HTML document.
        # Parts will be added every time the form (setHtmlForm below) is submitted (see handleForm).
        self.setHtml(TASKS_ID, id, html);

        # Test of gui elements
        # Messages will be added to the document when events happen (see handleEvent).
        #self.m_display.addCheckBox("v11n.python.setHtml", "cb.test.onoff", "Test On Off");
        #self.m_display.addButton("v11n.python.setHtml", "button.test", "Test Button");

    def remove_task(self, task):
        id = "%04d" % task.id
        self.setHtml(TASKS_ID, id, "");

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
