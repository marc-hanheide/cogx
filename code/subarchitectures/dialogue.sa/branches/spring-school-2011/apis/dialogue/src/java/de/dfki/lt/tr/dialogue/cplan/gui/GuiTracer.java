package de.dfki.lt.tr.dialogue.cplan.gui;

import java.util.concurrent.Semaphore;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.Rule;
import de.dfki.lt.tr.dialogue.cplan.RuleTracer;

public class GuiTracer implements RuleTracer {

  private int _ruleTracing = 0;
  private UPMainFrame _mf = null;

  public GuiTracer(UPMainFrame mainframe, int bitmask) {
    setTracing(bitmask);
    _mf = mainframe;
  }

  @Override
  public void traceMatch(DagEdge current, Rule r, Bindings bindings) {
    if ((_ruleTracing & DISPLAY_MATCHING) != 0) {
      _mf.setStatusLine(r.appendMatches(new StringBuilder()).toString());
      _mf.setInputDisplay(current.getValue());
    }
  }

  @Override
  public void traceBeforeApplication(DagEdge current, Rule r) {
    if ((_ruleTracing & DISPLAY_MODIFICATION) != 0) {
      _mf.setStatusLine("APPLY  " +
          r.appendActions(new StringBuilder()).toString());
      _mf.setInputDisplay(current.getValue());
    }
  }

  @Override
  public void traceAfterApplication(DagEdge current, Rule r) {
    // TODO mark what has been changed, or not
    if ((_ruleTracing & DISPLAY_MODIFICATION) != 0) {
      _mf.setOutputDisplay(current.getValue());
      Semaphore toWaitFor = new Semaphore(1);
      _mf.waitForStepTrace(toWaitFor);
      try {
        toWaitFor.acquire();
      } catch (InterruptedException e) {
      }
    }
  }

  @Override
  public void setTracing(int bitmask) {
    _ruleTracing = bitmask;
  }
 }
