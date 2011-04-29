package de.dfki.lt.tr.dialogue.cplan;

public interface RuleTracer {
  public static final int DISPLAY_MATCHING = 1;
  public static final int DISPLAY_MODIFICATION = DISPLAY_MATCHING << 1;

  public abstract void setTracing(int bitmask);

  public abstract void traceMatch(DagEdge current, Rule r,
      Bindings bindings);

  public abstract void traceAfterApplication(DagEdge current, Rule r);

  public abstract void traceBeforeApplication(DagEdge current, Rule r);

}
