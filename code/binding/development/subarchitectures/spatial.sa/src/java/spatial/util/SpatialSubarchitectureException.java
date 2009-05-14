/**
 * 
 */
package spatial.util;

import cast.architecture.subarchitecture.SubarchitectureProcessException;


/**
 * @author nah
 *
 */
public class SpatialSubarchitectureException extends SubarchitectureProcessException {

    /**
     * 
     */
    private static final long serialVersionUID = 7007958621076813030L;

    /**
     * 
     */
    public SpatialSubarchitectureException() {
        super();
    }

    /**
     * @param _message
     */
    public SpatialSubarchitectureException(String _message) {
        super(_message);
    }

    /**
     * @param _message
     * @param _cause
     */
    public SpatialSubarchitectureException(String _message,
            Throwable _cause) {
        super(_message, _cause);
    }

    /**
     * @param _cause
     */
    public SpatialSubarchitectureException(Throwable _cause) {
        super(_cause);
    }

}
