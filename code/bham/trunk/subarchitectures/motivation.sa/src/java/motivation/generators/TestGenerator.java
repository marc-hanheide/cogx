/**
 * 
 */
package motivation.generators;

import motivation.autogen.Motive;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 *
 */
public class TestGenerator extends Generator {
	@Override
	protected void runComponent() {
	    println("Look out world, here I come...");
	
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class, WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
        	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
        		System.out.println(CASTUtils.toString(_wmc));
        		try {
					System.out.println(getMemoryEntry(_wmc.address, Motive.class).goal);
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        	}
        }
        );

        Motive ann = new Motive("Hello World!");
        try {
        	addToWorkingMemory(newDataID(), ann);
        } catch (AlreadyExistsOnWMException e) {
        	e.printStackTrace();
        }

	}	
}
