/**
 * 
 */
package planning.util;


/**
 * @author nah
 *
 */
public class PlanningException extends Exception {

    /**
     * 
     */
    private static final long serialVersionUID = -5888911299131105154L;

    public PlanningException() {
        super();
    }

    public PlanningException(String _message, Throwable _cause) {
        super(_message, _cause);
    }

    public PlanningException(String _message) {
        super(_message);
    }

    public PlanningException(Throwable _cause) {
        super(_cause);
    }

}
