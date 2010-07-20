package de.dfki.lt.tr.dialogue.cplan.io;

import gnu.trove.TObjectIntHashMap;
import de.dfki.lt.tr.dialogue.cplan.DagNode;

public interface DagPrinter {

  public abstract void maxCoref(int corefs);

  public abstract void toStringRec(DagNode dag, boolean readable,
      StringBuilder sb, TObjectIntHashMap<DagNode> corefs) ;

}
