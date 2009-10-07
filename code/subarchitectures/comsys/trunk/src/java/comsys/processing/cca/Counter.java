package comsys.processing.cca;

import java.util.*;

public class Counter {

	protected String name;
	protected HashMap<String, Integer> subcounters = null;

	private final static int FIRST_INDEX = 1;
	
	public Counter(String _name) {
		name = _name;
		subcounters = new HashMap<String, Integer>();
	}
	
	public String inc(String subcounter) {
		
		Integer current = subcounters.get(subcounter);
		
		if (current == null) {
			current = new Integer(FIRST_INDEX);
		}
		else {
			current += 1;
		}
		subcounters.put(subcounter, current);

		return this.name + ":" + subcounter + ":" + current.toString();  
	}
}
