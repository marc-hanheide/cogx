package coma.aux;

import java.util.Set;
import java.util.HashSet;

/**
 * This class is a collection of static helper methods.
 * 
 * @author Hendrik Zender (zender@dfki.de)
 * @version 2008-07-07
 */
public final class ComaHelper {
    
	/**
	 * A helper method for converting the first letter of a String
	 * to lower case.
	 * 
	 * @param s - the input string
	 * @return the input string with a lower case first letter
	 */
	public static String firstToLower(String s) {
        if (s.length() == 0) return s;
        return s.substring(0, 1).toLowerCase() + s.substring(1);
    }

	/**
	 * A helper method for making sure a String has an initial
	 * capital letter and the rest is in lower case.
	 * 
	 * @param s - the input string
	 * @return the input string with a capped 1st letter and rest lower case
	 */
	public static String firstCapRestSmall(String s) {
        if (s.length() == 0) return s;
        return s.substring(0, 1).toUpperCase() + s.substring(1).toLowerCase();
    }

	/**
	 * A helper method for making sure a String has an initial
	 * capital letter and the rest stays as it is.
	 * 
	 * @param s - the input string
	 * @return the input string with a capped 1st letter
	 */
	public static String firstCap(String s) {
        if (s.length() == 0) return s;
        return s.substring(0, 1).toUpperCase() + s.substring(1);
    }
	
	
	public static <T> HashSet<T> computeSetDifference(Set<T> setA, Set<T> setB) {
	    HashSet<T> tmp = new HashSet<T>(setA);
	    tmp.removeAll(setB);
	    return tmp;
	  }
	
}
