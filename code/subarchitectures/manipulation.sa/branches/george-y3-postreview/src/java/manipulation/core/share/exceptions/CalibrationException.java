package manipulation.core.share.exceptions;

/**
 * Exception for the calibration procedure
 * 
 * @author Torben Toeniges
 * 
 */
public class CalibrationException extends Exception {

	private static final long serialVersionUID = 2213830222095707636L;

	/**
	 * exception constructor
	 */
	public CalibrationException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public CalibrationException(String s) {
		super(s);
	}
}
