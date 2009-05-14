/**
 * 
 */
package visionsa.wm;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;


/**
 * 
 * Placeholder for easy java testing. Real one will be written in C++.
 * 
 * @author nah
 *
 */
public class SimpleVisionWM extends SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public SimpleVisionWM(String _id) {
        super(_id);
        
        // determines whether this wm should broadcast to oher
        // sub-architectures
        setSendXarchChangeNotifications(true);
    }

}
