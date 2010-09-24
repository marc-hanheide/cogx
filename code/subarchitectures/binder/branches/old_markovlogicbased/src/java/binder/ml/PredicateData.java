package binder.ml;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * @author Carsten Ehrler
 * 
 */
public class PredicateData {
	/**
	 * This map keeps track of all names that appear for each
	 * predicate defined in Feature and M for each belief indexed by
	 * its belief_id
	 */
	public Map<Predicate, Set<String>> predicates_for_belief;
	public Map<String, Set<String>> values_for_types;

	public PredicateData() {
		predicates_for_belief = new TreeMap<Predicate, Set<String>>();
		values_for_types = new TreeMap<String, Set<String>>();
	}
	
	public boolean existsPredicate(Predicate predicate) {
		return predicates_for_belief.containsKey(predicate);
	}
	
	public void addValueForType(String type, String value) throws MLException {
		if(!values_for_types.containsKey(type)) {
			throw new MLException("no such type - add predicate with type first");
		}
		values_for_types.get(type).add(value);
	}
	
	public void addNewPredictate(String belief_id, Predicate predicate) throws MLException {
		if(predicates_for_belief.containsKey(predicate)) {
			throw new MLException("predicate already exists");
		}
		else {
			Set<String> beliefs = new TreeSet<String>();
			beliefs.add(belief_id);
			predicates_for_belief.put(predicate, beliefs);
		}
		
		for(String type : predicate.getArgumentTypes()) {
			if(!values_for_types.containsKey(type)) {
				Set<String> names = new TreeSet<String>();
				values_for_types.put(type, names);
			}
		}
	}
	
	public void addPredicateToBelief(String belief_id, Predicate predicate) throws MLException {
		if(!predicates_for_belief.containsKey(predicate)) {
			throw new MLException("predicate does not exist or was not added yet");
		}
		predicates_for_belief.get(predicate).add(belief_id);
	}
	
	public void removeBelief(String belief_id) {
		for(Set<String> beliefs : predicates_for_belief.values()) {
			beliefs.remove(belief_id);
		}
		
		List<Predicate> to_remove = new LinkedList<Predicate>();
		
		Set<String> types_to_remove = new TreeSet<String>();
		Set<String> types_to_keep = new TreeSet<String>();
		
		for(Predicate predicate : predicates_for_belief.keySet()) {
			if(predicates_for_belief.get(predicate).size() == 0) {
				to_remove.add(predicate);
				for(String type : predicate.getArgumentTypes())
					types_to_remove.add(type);
			}
			else {
				for(String type : predicate.getArgumentTypes())
				types_to_keep.add(type);
			}
		}
		
		// finally we remove all the predicates that do not appear anymore
		for(Predicate predicate : to_remove) {
			predicates_for_belief.remove(predicate);
		}
		
		// next we check if some types are not needed anymore and thus
		// can also be removed
		types_to_remove.removeAll(types_to_keep);
		
		for(String type : types_to_remove) {
			values_for_types.remove(type);
		}
	}

	public boolean hasPredicate(String predicate) {
		for(Predicate pred : predicates_for_belief.keySet()) {
			if(pred.getName().equals(predicate)) {
				return true;
			}
		}
		return false;
	}

	public boolean hasType(String string) {
		return values_for_types.containsKey(string);
	}

	public boolean hasBeliefForPredicate(String predicate, String belief_id) {
		for(Predicate pred : predicates_for_belief.keySet()) {
			if(pred.getName().equals(predicate)) {
				return predicates_for_belief.get(pred).contains(belief_id);
			}
		}
		return false;
	}

	public boolean hasValueForType(String type, String value) {
		return values_for_types.get(type).contains(value);
	}

	public Set<String> getValuesForType(String type) {
		return values_for_types.get(type);
	}

	public String getTypes() {
		StringBuilder types = new StringBuilder();
		
		for(String type : values_for_types.keySet()) {
			types.append(type);
			types.append(" = { ");
			Iterator<String> iter = values_for_types.get(type).iterator();
			
			if(iter.hasNext()) {
				types.append(iter.next());
			}
			
			while(iter.hasNext()) {
				types.append(" , ");
				types.append(iter.next());
			}
			types.append(" }\n");
		}
		return types.toString();
	}

	public String getDeclarations() {
		StringBuilder declarations = new StringBuilder(); 
		for(Predicate predicate : predicates_for_belief.keySet()) {
			declarations.append(predicate.getName());
			declarations.append("(");
			
			String[] args = predicate.getArgumentTypes();
			
			if(args.length > 0) {
				declarations.append(args[0]);
			}
			for(int i = 1; i < args.length; ++i) {
				declarations.append(", ");
				declarations.append(args[i]);
				declarations.append("!");
			}
			
			declarations.append(")\n");
		}
		return declarations.toString();
	}
}