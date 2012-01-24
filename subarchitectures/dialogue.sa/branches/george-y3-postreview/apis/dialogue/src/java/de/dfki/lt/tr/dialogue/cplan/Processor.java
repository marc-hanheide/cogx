package de.dfki.lt.tr.dialogue.cplan;

public interface Processor {
  public abstract DagNode applyRules(DagNode lf, Bindings bindings)
  throws InterruptedException;
}
