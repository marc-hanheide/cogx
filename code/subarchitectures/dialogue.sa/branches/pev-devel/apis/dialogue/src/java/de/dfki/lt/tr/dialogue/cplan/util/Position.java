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

  @Override
  public boolean equals(Object o) {
    if (! (o instanceof Position)) return false;
    Position p2 = (Position) o;
    return line == p2.line && column == p2.column && msg.equals(p2.msg);
  }

  @Override
  public String toString() {
    return msg + ":" + line + ":" + column;
  }
}
