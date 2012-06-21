package de.dfki.lt.tr.dialogue.cplan;

import de.dfki.lt.tr.dialogue.cplan.matches.FunCall;
import de.dfki.lt.tr.dialogue.cplan.matches.GlobalVar;
import de.dfki.lt.tr.dialogue.cplan.matches.Match;
import de.dfki.lt.tr.dialogue.cplan.matches.MatchLVal;

public class VarMatch {
  private MatchLVal _lval;
  private Match _match;

  public VarMatch(MatchLVal lval, Match match) {
    _lval = lval;
    _match = match;
  }

  public DagEdge getLVal(Bindings bindings) {
    DagEdge current = null;
    if (_lval instanceof GlobalVar) {
      current = ((GlobalVar)_lval).getBinding(null, bindings);
      if (current == null) {
        current = new DagEdge((short)-1, new DagNode());
      }
    }
    if (_lval instanceof FunCall) {
      current = ((FunCall)_lval).getBinding(null, bindings);
    }
    return current;
  }

  protected boolean match(DagEdge here, Bindings bindings) {
    if (_lval == null) {
      if (_match.startMatch(here, bindings)) {
        // bind the current location locally to "#"
        bindings.bind("#", here, Bindings.LOCAL);
        return true;
      }
      return false;
    }
    return _match.startMatch(getLVal(bindings), bindings);
  }

  @Override
  public String toString() {
    if (_lval == null) {
      return _match.toString();
    }
    return "\n, " + _lval + " ^ " + _match;
  }
}
