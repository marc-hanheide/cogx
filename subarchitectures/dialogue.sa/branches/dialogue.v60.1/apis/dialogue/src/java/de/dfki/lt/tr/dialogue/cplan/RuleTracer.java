package de.dfki.lt.tr.dialogue.cplan;

import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.actions.Action;

public interface RuleTracer {
  public static final int DISPLAY_MATCHING = 1;
  public static final int DISPLAY_MODIFICATION = DISPLAY_MATCHING << 1;

  public abstract void traceMatch(DagEdge current, List<VarMatch> matches,
      Bindings bindings);

  public abstract void traceAfterApplication(DagEdge current, Action action);

  public abstract void traceBeforeApplication(DagEdge current, Action action);

}
