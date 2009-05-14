package org.cognitivesystems.reasoner.base;

/**
 * This interface defines what methods an Instance implementation should provide.
 * 
 * @author Hendrik Zender, zender@dfki.de
 * @version June 28, 2007
 */
public interface ReasonerInstance extends Comparable {

	/**
	 * @return the short name of the Instance
	 */
	public abstract String getName();
	
	/**
	 * @return the full name of the Instance
	 */
	public abstract String getFullName();

	/**
	 * @return the full name of the Instance
	 */
	public abstract String getNamespace();
	
	/**
	 * @return the separator of the Instance
	 */
	public abstract String getSep();
	
}
