package de.dfki.lt.tr.dialogue.cplan.matches;


import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;

/** Abstract superclass of all syntax tree nodes for the parsing of graph
 *  transformation matching rules.
 */
public abstract class Match {
  protected boolean _negated = false;

  /** See if, given the variable bindings, all conditions specified in this
   *  subtree can be fulfilled on the given input node
   */
  protected abstract boolean match(DagEdge input, Bindings bindings);

  /** This method keeps track of the embedding of the matches which is important
   *  when a local failure occurs. In this case, all bindings that have been
   *  created in the meantime have to be retracted. This is the only method
   *  that is allowed to call match directly.
   */
  protected boolean matches(DagEdge input, Bindings bindings) {
    int currentLevel = bindings.getLevel();
    boolean result = match(input, bindings);
    if (_negated) result = ! result;
    if (! result) {
      // in case of local failure, we have to retract the variable bindings
      // that were established in that subtree
      bindings.retractToLevel(currentLevel);
    }
    return result;
  }

  public boolean startMatch(DagEdge input, Bindings bindings) {
    boolean result = matches(input, bindings);
    // We have a global success, so make the remaining bindings permanent
    return result;
  }

  public void setNegated(boolean value) {
    _negated = value;
  }

  /** Convert this subtree into normal form. At the moment, this only converts
   *  trees of directly nested conjunctions into a leftist tree, i.e., a chain
   *  of conjunctions. This might be extended in the future to transform
   *  the tree into disjunctive normal form.
   */
  void normalForm() {}

  protected DagNode newDag() {
    return new DagNode(); // empty dag with type TOP
  }

  protected DagNode newDag(String typename) {
    return new DagNode(typename);
  }

}

