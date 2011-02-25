package manipulation.core.share.exceptions;

/**
 * Exeption for view point failures
 * 
 * @author ttoenige
 * 
 */
public class ViewPointException extends Exception {

	private static final long serialVersionUID = -7429860254342125599L;

	/**
	 * exception constructor
	 */
	public ViewPointException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public ViewPointException(String s) {
		super(s);
	}
}
