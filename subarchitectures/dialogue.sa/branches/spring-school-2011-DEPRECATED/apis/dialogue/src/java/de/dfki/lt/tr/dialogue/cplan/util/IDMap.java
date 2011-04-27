package de.dfki.lt.tr.dialogue.cplan.util;

import java.util.ArrayList;

/** map things from and to integer IDs. Since we need both directions, this
 *  functionality is integrated here instead of using two containers.
 */
public abstract class IDMap<THING> {
  /** data structure to map from the id to the corresponding thing */
  protected ArrayList<THING> number2Thing;

  public IDMap() {
    number2Thing = new ArrayList<THING>();
  }

  public IDMap(int initialCapacity) {
    number2Thing = new ArrayList<THING>(initialCapacity);
  }

  final protected int registerNext(THING thing) {
    number2Thing.add(thing);
    return number2Thing.size() - 1;
  }

  public THING fromId(int id) {
    if (id >= number2Thing.size()) return null;
    return number2Thing.get(id);
  }

  public abstract boolean contains(THING thing);

  public int size() { return number2Thing.size(); }
}
