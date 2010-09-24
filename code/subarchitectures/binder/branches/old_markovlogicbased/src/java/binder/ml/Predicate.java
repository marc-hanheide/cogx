package binder.ml;

public class Predicate implements Comparable<Object>{
	
	private String name;
	private String[] argument_types;
	
	public Predicate(String name, String[] argument_types) {
		this.name = name;
		this.argument_types = argument_types; 
	}
	
	public Predicate(String name, String second_type) {
		this.name = name;
		this.argument_types = new String[2];
		this.argument_types[0] = "belief_id";
		this.argument_types[1] = second_type;
		
	}
	
	public String getName() {
		return name;
	}
	 
	public String[] getArgumentTypes() {
		return argument_types;
	}
	
	public int compareTo(Object other) {
		if(other == null) {
			return 1;
		}
		if(!(other instanceof Predicate)) {
			return 1;
		}
		Predicate that = (Predicate)other;
		
		return this.name.compareTo(that.name);
	}
}
