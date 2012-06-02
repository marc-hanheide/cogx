package cdsr.data;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import cdsr.objects.AnnotatedCDSR;

/**
 * 
 * @author GSH
 *
 */
public class SubjectDataHolder
{

  // type (e.g. front) to room (e.g. 222) to Set of AnnotatedCDSRs
  private Map<String, Map<String, Set<AnnotatedCDSR>>> typeToRoomToSetOfAnnotatedCDSRs;

  public SubjectDataHolder()
  {
    typeToRoomToSetOfAnnotatedCDSRs = new HashMap<String, Map<String, Set<AnnotatedCDSR>>>();
  }

  /**
   * 
   * @return a set of all CDSR types for which there are results
   */
  public Set<String> getTypes()
  {
    return typeToRoomToSetOfAnnotatedCDSRs.keySet();
  }

  /**
   * 
   * @param _type
   *          The CDSR type, e.g. front
   * @return a map from room to a set of AnnotatedCDSRs
   */
  public Map<String, Set<AnnotatedCDSR>> getResultsForType(String _type)
  {
    return typeToRoomToSetOfAnnotatedCDSRs.get(_type);
  }

  /**
   *  Takes the set of results and puts them into the appropriate place
   *  in the map for easier access later.
   * @param _results a set of AnnotatedCDSRs
   */
  public void sortResults(Set<AnnotatedCDSR> _results)
  {
    Iterator<AnnotatedCDSR> itr = _results.iterator();
    while (itr.hasNext())
    {
      AnnotatedCDSR next = itr.next();
      Map<String, Set<AnnotatedCDSR>> roomToCdsrs = typeToRoomToSetOfAnnotatedCDSRs
          .get(next.getType());
      if (roomToCdsrs == null)
      {
        roomToCdsrs = new HashMap<String, Set<AnnotatedCDSR>>();
      }
      Set<AnnotatedCDSR> cdsrs = roomToCdsrs.get(next.getRoom());
      if (cdsrs == null)
      {
        cdsrs = new HashSet<AnnotatedCDSR>();
      }
      cdsrs.add(next);
    }
  }

}
