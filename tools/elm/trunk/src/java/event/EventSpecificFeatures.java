/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.util.Set;
import java.util.Iterator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Collection;

public class EventSpecificFeatures extends HashMap<String, HashSet<String>> {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public EventSpecificFeatures() {
	}

	public EventSpecificFeatures(int initialCapacity) {
		super(initialCapacity);
	}

// 	public void addKeyValuePair(String key, Object value) {
// 
// 		HashSet<String> v = get(key);
// 
// 		if (v == null) {
// 			v = new HashSet<String>();
// 			v.add(value.toString());
// 			put(key, v);
// 		} else
// 			v.add(value.toString());
// 	}

	public void addKeyValuePair(String key, String value) {

		HashSet<String> v = get(key);

		if (v == null) {
			v = new HashSet<String>();
			v.add(value);
			put(key, v);
		} else
			v.add(value);
	}
	
	
	public boolean containsKeyValuePair(String key, String value) {

		HashSet<String> v = get(key);

		if (v == null)
			return false;
		else
			return v.contains(value);
	}

	public void put(String key, String[] values) {

		HashSet<String> hs = new HashSet<String>(values.length);
		for (String v : values)
			hs.add(v);

		if (hs.size() > 0)
		    put(key, hs);
	}

	public void put(String key, Collection<String> values) {
		if (values.size() > 0)
		    put(key, new HashSet<String>(values));
	}

	/**
	 *  Number of (feature name, feature value) pairs.
	 */
	public int totalNumberOfPairs() {
	    
		int esfFeaturesSum = 0;
		
		Iterator<String> keyIterator = keySet().iterator();
	
		while (keyIterator.hasNext()) {
		    Set<String> values = get(keyIterator.next());
		    Iterator<String> valuesIterator = values.iterator();
		    while (valuesIterator.hasNext()) {
			valuesIterator.next();
			esfFeaturesSum++; 
		    }
		}
		return esfFeaturesSum;
	}	
}
