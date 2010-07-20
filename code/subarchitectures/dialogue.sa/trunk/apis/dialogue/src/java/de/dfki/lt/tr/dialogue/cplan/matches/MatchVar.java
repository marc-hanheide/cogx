package de.dfki.lt.tr.dialogue.cplan.matches;

import de.dfki.lt.tr.dialogue.cplan.Bindings;
import de.dfki.lt.tr.dialogue.cplan.DagEdge;

public abstract class MatchVar extends Match {

  public abstract DagEdge getBinding(DagEdge input, Bindings bindings);

}
