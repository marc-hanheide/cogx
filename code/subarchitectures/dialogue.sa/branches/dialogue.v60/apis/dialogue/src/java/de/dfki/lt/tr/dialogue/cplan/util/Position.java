package de.dfki.lt.tr.dialogue.cplan.util;

public class Position {
  public Position(int line, int column, String msg) {
    this.line = line;
    this.column = column;
    this.msg = msg;
  }

  public int line;
  public int column;
  public String msg;
}
