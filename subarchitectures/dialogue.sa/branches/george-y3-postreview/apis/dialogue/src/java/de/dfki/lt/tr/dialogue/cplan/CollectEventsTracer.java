package de.dfki.lt.tr.dialogue.cplan;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** This tracer only works if the actions are performed in the same order as
 *  the matches.
 */
public class CollectEventsTracer extends AbstractTracer {

  private List<TraceEvent> _events;

  int _last = 0;

  /** Initialize a new tracer with internal event storage */
  public CollectEventsTracer() {
    this(new ArrayList<TraceEvent>());
  }

  /** Initialize a new tracer with external event storage */
  public CollectEventsTracer(List<TraceEvent> coll) {
    _events = coll;
  }

  public List<TraceEvent> getEvents() {
    return _events;
  }

  @Override
  public void traceAfterApplication(DagEdge root, DagEdge current, Rule r,
      Bindings bindings) {
    TraceEvent curr = _events.get(_last);
    curr.bindings = bindings.copy();
    // we copy the intermediate dag, and get a mapping from original
    // to the copied edges
    Map<Object, Object> origToCopy = new HashMap<Object, Object>();
    curr.curr = root.copyIntermediate(origToCopy).getValue();
    curr.appPoint = (DagEdge) origToCopy.get(current); // or arg: curr.appPoint??
    ++_last;
  }

  @Override
  public void traceMatch(DagEdge root, DagEdge current, Rule r,
      Bindings bindings) throws InterruptedException {
    checkInterrupt();
    TraceEvent curr = new TraceEvent();
    curr.rule = r;
    curr.appPoint = current;
    curr.lastMatch = root.getValue().cloneFS();
    _events.add(curr);
  }
}
