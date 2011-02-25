package manipulation.core.share.exceptions;

/**
 * Exception for mathematical failures
 * 
 * @author ttoenige
 * 
 */
public class MathException extends Exception {

	private static final long serialVersionUID = 8319225516246380095L;

	/**
	 * exception constructor
	 */
	public MathException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public MathException(String s) {
		super(s);
	}
}
