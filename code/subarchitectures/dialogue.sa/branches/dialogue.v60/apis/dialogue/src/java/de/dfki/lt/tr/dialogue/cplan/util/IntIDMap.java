package de.dfki.lt.tr.dialogue.cplan.util;

import gnu.trove.TObjectIntHashMap;


/** map things from and to integer IDs. Since we need both directions, this
 *  functionality is integrated here instead of using two containers.
 */
public class IntIDMap<THING> extends IDMap<THING> {
  /** data structure to map from the thing to the corresponding id */
  protected TObjectIntHashMap<THING> type2Number;

  private static final int ILLEGAL_VALUE = -1;

  public IntIDMap() {
    super();
    type2Number = new TObjectIntHashMap<THING>();
  }

  public IntIDMap(int initialCapacity) {
    super(initialCapacity);
    type2Number = new TObjectIntHashMap<THING>(initialCapacity);
  }

  public int register(THING thing) {
    int newID = registerNext(thing);
    type2Number.put(thing, newID);
    return newID;
  }

  public int getId(THING thing) {
    return (type2Number.contains(thing) ? type2Number.get(thing)
                                        : ILLEGAL_VALUE);
  }

  @Override
  public boolean contains(THING thing) {
    return type2Number.contains(thing);
  }
  @Override
  @SuppressWarnings("unchecked")
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("{ ");
    for (Object o : type2Number.keys()) {
      THING t = (THING) o;
      sb.append("{" + t + "," + type2Number.get(t) + "} ");
    }
    sb.append("}");
    return sb.toString();
  }
}
