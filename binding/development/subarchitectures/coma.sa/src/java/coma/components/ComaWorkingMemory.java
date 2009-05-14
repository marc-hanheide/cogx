package coma.components;

import com.sun.corba.se.spi.legacy.connection.GetEndPointInfoAgainException;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;

/**
 * Conceptual Map SA WM
 * 
 * @author zender, based on nah's StageWorkingMemory and input from henrikj, maria
 */
public class ComaWorkingMemory extends SubarchitectureWorkingMemory {



    /**
     * @param _id
     */
    public ComaWorkingMemory(String _id) {
        super(_id);
        setSendXarchChangeNotifications(true);
    }

    /*
     * (non-Javadoc)
     * 
     * @see framework.core.processes.FrameworkProcess#run()
     */
    @Override
    public void runComponent() {
    	// handy for debugging!
    	while (m_status == ProcessStatus.RUN) {
    		try {
    			m_semaphore.acquire();
    			//just print out memory contents
    			//log(m_workingMemory);

    			m_semaphore.release();
    			Thread.sleep(10000);
    		}
    		catch (InterruptedException e) {
    			e.printStackTrace();
    		}
    	}
    }
}
