package de.dfki.lt.tr.dialogue.cplan.matches;


import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;

/** Abstract superclass of all syntax tree nodes for the parsing of graph
 *  transformation matching rules.
 */
public abstract class Match {

  /** See if, given the variable bindings, all conditions specified in this
   *  subtree can be fulfilled on the given input node
   */
  public abstract boolean match(DagEdge input, Bindings bindings);

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

