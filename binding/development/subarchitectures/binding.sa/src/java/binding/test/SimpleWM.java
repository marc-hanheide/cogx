package binding.test;
import cast.architecture.SubarchitectureWorkingMemory;

public class SimpleWM extends SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public SimpleWM(String _id) {
        super();
        setSendXarchChangeNotifications(true);
    }
}
