package de.dfki.lt.tr.dialogue.cplan;

public class TraceEvent {
  /** The state of the DagNode in the last matching phase */
  public DagNode lastMatch;
  /** The current state of the DagNode after the last application */
  public DagNode curr;
  /** What edge is the action applied to? */
  public DagEdge appPoint;
  /** What is the rule that is matched / applied */
  public Rule rule;
  /** What were the bindings when the match was performed */
  public Bindings bindings;
}
