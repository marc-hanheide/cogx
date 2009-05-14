package coma.aux;

import cast.cdl.TriBool;

/**
 * This class is a collection of static helper methods.
 * 
 * @author Hendrik Zender (zender@dfki.de)
 * @version 2008-07-07
 */
public final class ComaHelper {

	
    /**
     * This is a helper method for producing a String of a TriBool value.
     * 
     * @param _tribool
     * @return a String representation of the given TriBool
     */
    public static String triBool2String(TriBool _tribool) {
        switch (_tribool.value()) {
            case TriBool._triFalse:
                return "triFalse";
            case TriBool._triTrue:
                return "triTrue";
            case TriBool._triIndeterminate:
                return "triIndeterminate";
            default:
                break;
        }
        return "Error converting TriBool!";
    }
    
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


	
}
