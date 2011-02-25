package manipulation.core.share.exceptions;

/**
 * Exception for failures while generating maps
 * 
 * @author ttoenige
 * 
 */
public class MapException extends Exception {
	private static final long serialVersionUID = -3247641538975525183L;

	/**
	 * exception constructor
	 */
	public MapException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public MapException(String s) {
		super(s);
	}
}
