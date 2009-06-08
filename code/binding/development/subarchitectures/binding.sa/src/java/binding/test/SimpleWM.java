package binding.test;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;

public class SimpleWM extends SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public SimpleWM(String _id) {
        super(_id);
        setSendXarchChangeNotifications(true);
    }
}
