package de.dfki.lt.tr.dialogue.cplan.matches;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;

/** A syntax tree node representing the conjunction between two subnodes */
public class Conjunction extends Match {
  private Match _left, _right;

  public Conjunction(Match left, Match right) {
    _left = left;
    _right = right;
  }

  @Override
  public String toString() {
    return (_negated ? "!" : "") + "(" + _left + " ^ " + _right + ")";
  }

  @Override
  protected boolean match(DagEdge input, Bindings bindings) {
    return _left.matches(input, bindings) && _right.matches(input, bindings);
  }

  private void collectRec(List<Match> leaves, List<Conjunction> nodes){
    nodes.add(this);
    if (_left instanceof Conjunction) {
      ((Conjunction)_left).collectRec(leaves, nodes);
    } else {
      leaves.add(_left);
    }
    if (_right instanceof Conjunction) {
      ((Conjunction)_right).collectRec(leaves, nodes);
    } else {
      leaves.add(_right);
    }
  }

  /** change a chain of conjunctions in the following way:
   *  a) it is a rightist tree: all left branches are not conjunctions,
   *     and all but the bottom right branches are conjuncts
   *  b) if there is a global var somewhere on the fringe, it's moved into
   *     the first left branch to make the whole thing a separate "global" match
   */
  @Override
  void normalForm() {
    List<Match> leaves = new ArrayList<Match>();
    List<Conjunction> nodes = new ArrayList<Conjunction>();
    collectRec(leaves, nodes);
    Collections.sort(leaves, new Comparator<Match>() {
      public int compare(Match a, Match b) {
        if (a instanceof GlobalVar) return -1;
        if (b instanceof GlobalVar) return 1;
        return 0;
      }
    }
    );
    int i = 0;
    for (; i < nodes.size() - 1; ++i) {
      nodes.get(i)._left = leaves.get(i);
      nodes.get(i)._right = nodes.get(i + 1);
    }
    nodes.get(i)._right = leaves.get(i + 1);
  }
}
