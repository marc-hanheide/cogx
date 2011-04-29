package de.dfki.lt.tr.dialogue.cplan.util;

import gnu.trove.TObjectShortHashMap;


/** map things from and to integer IDs. Since we need both directions, this
 *  functionality is integrated here instead of using two containers.
 */
public class ShortIDMap<THING> extends IDMap<THING> {
  /** data structure to map from the thing to the corresponding id */
  protected TObjectShortHashMap<THING> type2Number;

  public ShortIDMap() {
    super();
    type2Number = new TObjectShortHashMap<THING>();
  }

  public ShortIDMap(int initialCapacity) {
    super(initialCapacity);
    type2Number = new TObjectShortHashMap<THING>(initialCapacity);
  }

  public short register(THING thing) {
    short newID = (short) registerNext(thing);
    type2Number.put(thing, newID);
    return newID;
  }

  public short getId(THING thing) {
    return type2Number.get(thing);
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
      sb.append("{" + o + "," + type2Number.get(t) + "} ");
    }
    sb.append("}");
    return sb.toString();
  }
}
