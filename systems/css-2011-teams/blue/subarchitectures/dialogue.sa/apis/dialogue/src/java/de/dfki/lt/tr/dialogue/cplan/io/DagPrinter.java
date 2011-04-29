package de.dfki.lt.tr.dialogue.cplan.io;

import java.util.IdentityHashMap;

import de.dfki.lt.tr.dialogue.cplan.DagNode;

public abstract class DagPrinter {
  private IdentityHashMap<DagNode, Integer> corefs;
  protected int _maxCoref;

  public abstract void toStringRec(DagNode dag, boolean readable,
      StringBuilder sb) ;

  public void getCorefs(DagNode root) {
    corefs = new IdentityHashMap<DagNode, Integer>();
    _maxCoref = root.countCorefsLocal(corefs, 0);
  }

  protected int getCorefNo(DagNode here) {
    int corefNo = corefs.get(here);
    if (corefNo > 0)
      corefs.put(here, -corefNo);
    return corefNo;
  }
}
