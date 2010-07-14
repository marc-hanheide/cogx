package de.dfki.lt.tr.dialogue.cplan;

import de.dfki.lt.tr.dialogue.cplan.matches.Match;

public class VarMatch {
  public String varName;
  public Match match;
  public VarMatch(String varName, Match match) {
    this.varName = varName;
    this.match = match;
  }
}
