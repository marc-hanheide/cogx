package manipulation.core.share.exceptions;

/**
 * Exception for failures during manipulation
 * 
 * @author Torben Toeniges
 * 
 */
public class ManipulatorException extends Exception {

	private static final long serialVersionUID = -8485933009615369842L;

	/**
	 * exception constructor
	 */
	public ManipulatorException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public ManipulatorException(String s) {
		super(s);
	}
}
