/**
 * 
 */
package planning.util;

import cast.architecture.subarchitecture.SubarchitectureProcessException;


/**
 * @author nah
 *
 */
public class PlanningSubarchitectureException extends SubarchitectureProcessException {

    /**
     * 
     */
    private static final long serialVersionUID = -1140459280106700210L;

    /**
     * 
     */
    public PlanningSubarchitectureException() {
        super();
    }

    /**
     * @param _message
     */
    public PlanningSubarchitectureException(String _message) {
        super(_message);
    }

    /**
     * @param _message
     * @param _cause
     */
    public PlanningSubarchitectureException(String _message,
            Throwable _cause) {
        super(_message, _cause);
    }

    /**
     * @param _cause
     */
    public PlanningSubarchitectureException(Throwable _cause) {
        super(_cause);
    }

}
